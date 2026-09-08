# Network scaling (protocol v3) — design lives in the tag repo

**Date:** 2026-09-07
**Status:** pointer

The scaling design for 100 tags / 100 m / 32 anchors is a **whole-network** design: it
changes the on-air contract, so it lives next to that contract in the tag repo rather
than being duplicated (and drifting) here.

- **Design:** `../../../../tag_testting/spec/2026-09-07-network-scaling-design.md`
- **On-air contract it supersedes (v2 -> v3):**
  `../../../../tag_testting/spec/2026-06-17-uwb-mac-protocol-contract.md`
- **This repo's implementation plan:** `../plans/2026-09-07-network-scaling-anchor.md`
- **Tag implementation plan:** `../../../../tag_testting/plan/2026-09-07-network-scaling-tag.md`

Anchor/gateway summary of what changes here: `UWB_MAX_ANCHORS` 4 -> 32 (wire cap 254),
grouped DISCOVERY responses (rank inside a group, not raw id, so the stagger stays
12.5 ms at any deployment size), a new `0xEC` ANNOUNCE emitted by one anchor per
superframe on a prime schedule, a `0xE3` MULTI-POLL responder answering with the new
`0xED` frame (which carries anchor `z`), and a gateway seat table of `16 x 14` = 224
phase-seats with leases renewed from `0xEA` POS frames.
