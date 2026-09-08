# Task 16 report: Short addresses are reused by EUI

## What was implemented

1. `struct gw_addr_map_entry` (src/gw_core.h): `{used, eui[8], short_addr, touch}`.
   `touch` is a monotonic counter stamped on insert/update, used only to find the
   oldest entry for eviction.
2. `GW_ADDR_MAP_SIZE` (64) and `struct gw_addr_map_entry addr_map[GW_ADDR_MAP_SIZE]`
   plus `uint32_t addr_map_touch_ctr` added to `struct gw_core_ctx`, deliberately
   NOT inside `seats[][]`/`struct gw_seat` so it survives the memset
   `gw_core_superframe_tick()` does on lease expiry. In-RAM only; `gw_core_init()`'s
   existing `memset(c, 0, sizeof(*c))` zeroes it along with everything else, no
   separate init needed.
3. Two static helpers in gw_core.c:
   - `addr_map_find()`: bounded 64-entry linear scan for a remembered address by EUI.
   - `addr_map_put()`: updates an existing entry for that EUI in place, else uses a
     free slot, else evicts the entry with the smallest `touch` (the oldest, since
     `touch` is monotonically increasing and stamped on every write) — bounded
     single scan that finds free-slot-or-oldest in one pass.
4. `gw_core_join()`'s new-seat branch (the one reached only when
   `find_seat_by_eui()` finds nothing — i.e. genuinely new EUI or an EUI whose
   seat already fully expired) now: checks `addr_map_find()` first; if found,
   defensively re-verifies via `find_seat_by_addr()` that the remembered address
   isn't currently live under some other EUI before reusing it (documented as
   should-be-unreachable, kept anyway — see self-review below); otherwise falls
   back to `alloc_short_addr()` exactly as before. Either way, `addr_map_put()`
   records the pairing after allocation.
5. `tag_id_from_eui()` / `pos_json.c` untouched, per the brief.

## Design reasoning

- **Map structure**: kept to the minimum the brief's three tests require — no
  wall-clock timestamps, no doubly-linked LRU list, just a monotonic counter and a
  linear min-scan. `GW_ADDR_MAP_SIZE` (64) is small enough that this is a trivial,
  bounded cost on the `K_PRIO_COOP(0)` gateway loop, consistent with every other
  bounded-scan helper already in this file (`find_seat_by_addr`,
  `find_seat_by_eui`, `alloc_short_addr`'s own guard loop).
- **Eviction key = insertion/update order, not last-JOIN-refresh order**: the
  brief's own suggestion ("a plain monotonic insertion/touch counter and evicting
  the minimum is sufficient") is exactly what's implemented. I deliberately did
  NOT wire in touching from KEEPALIVE/POS/live-rejoin, per "keep it as simple as
  the acceptance tests actually require" — only `gw_core_join()`'s allocation
  path reads and writes the map.
- **Placement in `gw_core_ctx`, not a separate global**: this repo's convention
  (per CLAUDE.md) is that gateway loop state lives in this file-scope static
  struct precisely to keep it off the loop's stack; a 64×~14-byte array here is
  the same pattern the existing `seats[GW_CYCLE_C][GW_N_CFP]` table already uses.

## Self-review

- **Can two different EUIs ever hold the same short address at once?** Traced
  carefully: the only place a remembered address is reused is right after
  `find_seat_by_eui(c, eui, ...)` (called at the top of `gw_core_join()`) has
  already returned false for THIS eui, meaning no live seat anywhere holds this
  EUI. The map lookup then finds a remembered `(eui -> addr)` pairing from some
  earlier allocation. Before trusting it, the code calls
  `find_seat_by_addr(c, remembered, ...)` — if ANY live seat (regardless of
  whose EUI it holds) currently owns that exact address, the code falls back to
  `alloc_short_addr()` instead of reusing it. So the reuse path can only ever
  hand out an address that is not currently held by anyone. Given
  `alloc_short_addr()` itself only ever returns addresses not currently live
  (its own `find_seat_by_addr` check), and `addr_map_put()` only records
  addresses that came from one of these two paths, I could not construct a
  scenario producing a collision. However, the defensive check is NOT unreachable
  in practice — it guards against address-pool wraparound. alloc_short_addr()
  is a bare monotonic counter that wraps at 0xFFFE back to GW_TAG_ADDR_BASE,
  checking only that an address isn't currently held by a live seat. After
  enough JOIN churn, the counter can wrap around and hand the same numeric
  address to a brand-new EUI, while the addr_map still remembers that address
  as belonging to a long-expired EUI (one whose seat expired without rejoin).
  The defensive check catches exactly this reachable scenario, verifying the
  remembered address isn't currently live under a different EUI before reusing
  it, and is cheap (one bounded scan already needed for other purposes) so it is
  kept rather than asserting the invariant silently.
- **Does the rejoin-after-full-lease-expiry test tick the lease to zero?** Yes —
  `test_rejoin_after_lease_expiry_reuses_address()` ticks exactly `GW_LEASE_SF`
  times and asserts `c.seats[...].short_addr == 0` (seat reclaimed) BEFORE
  rejoining and checking `g2.short_addr == g1.short_addr`. This is the real
  reclaim path (`gw_core_superframe_tick()`'s memset), not `gw_core_release()`.
- **Does the eviction test prove oldest-first, not just "something gets
  evicted"?** Yes. `test_addr_map_evicts_oldest_first()` inserts
  `GW_ADDR_MAP_SIZE + 1` (65) distinct EUIs in order (each joined then fully
  lease-expired, so each occupies exactly one map slot and never a seat). It
  then checks, in this order: (1) EUI 1 — the oldest entry that SURVIVED the
  first eviction — still recovers its exact original address, proving the
  eviction correctly picked EUI 0 (the true oldest) and left EUI 1 alone; then,
  after ticking EUI 1's rejoin-created seat back to expiry so it doesn't
  interfere, (2) EUI 0 — the entry actually evicted — gets a NEW address,
  proving it was NOT still remembered. Checking EUI 1 before touching EUI 0
  again was necessary: EUI 0's own rejoin re-inserts it into the map, which
  would otherwise evict EUI 1 in turn (now the new oldest) and mask the
  oldest-first proof — an ordering bug I hit and fixed while writing the test
  (first draft checked EUI 0 first and failed correctly, confirming the
  eviction logic — the test order was wrong, not the implementation).
- **Does a full map still allow new JOINs?** Yes —
  `test_addr_map_full_still_grants_fresh_address()` runs `GW_ADDR_MAP_SIZE + 4`
  (68) fresh EUIs through join/expire, past the map's capacity by several
  entries, and asserts every single JOIN succeeds with a nonzero address.
  `addr_map_put()` always finds either a free slot or an eviction candidate
  (the scan always finds SOME entry, since the map has 64 entries and any full
  map has all of them `used`), so it can never fail to make room; allocation
  itself only fails when the SEAT table (`alloc_phases()`) is full, unrelated
  to this map.
- **Test output pristine?** `gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe
  tests/gw_core/test_gw_core.c src/gw_core.c` — clean build, zero warnings.
  `./tests/gw_core/test_gw_core.exe` → `PASSED`.

## Zephyr/Xtensa compile check

Attempted using `build/compile_commands.json`'s recorded command for
`src/gw_core.c` with `-c` swapped for `-fsyntax-only` (with `$env:ZEPHYR_BASE`
set). It failed with an environment error unrelated to this change:
`fatal error: cannot read spec file 'picolibc': No such file or directory` —
the recorded command's `-specs=picolibc.specs` can't resolve outside a
`west build` invocation's working environment/PATH in this session. Did not
pursue further per the brief's allowance ("a full `west build` link is not
expected to succeed... otherwise note that in your report"); the host-test
build already exercises the exact same C source, and my changes are ordinary
portable C (no Zephyr APIs touched) so this is unlikely to hide a real issue.

## Files changed

- `src/gw_core.h` — `GW_ADDR_MAP_SIZE`, `struct gw_addr_map_entry`, map fields
  in `struct gw_core_ctx`.
- `src/gw_core.c` — `addr_map_find()`, `addr_map_put()`, integration into
  `gw_core_join()`'s new-seat branch.
- `tests/gw_core/test_gw_core.c` — three new tests plus their `main()` calls.

## Concerns

None outstanding. The one thing worth flagging for a reviewer: the defensive
`find_seat_by_addr()` re-check inside the reuse path is believed unreachable
(see self-review above) but is retained rather than asserted-away, per the
task brief's own suggestion.

## Fix round 1

**Finding:** The comment on the defensive `find_seat_by_addr()` check (src/gw_core.c
lines 521-532) incorrectly justified it as guarding an "unreachable" scenario where
"two EUIs collided on address assignment." The self-review section made the same
claim.

**Root cause:** Address-pool wraparound, not collision prevention. After enough
JOIN churn, `alloc_short_addr()`'s bare monotonic counter wraps at 0xFFFE back to
GW_TAG_ADDR_BASE. It only checks that an address isn't currently held by a live
seat. A long-expired EUI's entry in `addr_map` (which survives lease expiry) can
remain while the wrapped counter hands its same numeric address to a brand-new EUI.
The check is reachable and load-bearing under this condition.

**Before:** "...this should be unreachable in practice, since alloc_short_addr()
never hands out an address find_seat_by_addr() reports live, and addr_map only ever
records addresses this function itself allocated..."

**After:** "...The defensive find_seat_by_addr() check guards against address-pool
wraparound: after enough JOIN churn, alloc_short_addr()'s monotonic counter wraps
at 0xFFFE back to GW_TAG_ADDR_BASE...a stale map entry can validly receive the same
address a wrapped-around counter hands to a new EUI..."

**Files updated:**
- `src/gw_core.c` lines 521-532 — corrected comment to name address wraparound
- Self-review section (lines 61-69 of this report) — corrected to explain reachable
  scenario and confirm the check is load-bearing, not dead code

**Test status:** `gcc -Wall -Wextra -Isrc -o tests/gw_core/test_gw_core.exe
tests/gw_core/test_gw_core.c src/gw_core.c && ./tests/gw_core/test_gw_core.exe` →
PASSED. Comment-only change, logic untouched.
