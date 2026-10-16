# Edge TDoA server

Solves TDoA positions off-board and publishes them with the frozen platform payload,
so the gateway (ESP32) is left with the beacon, the CCP and the survey.

```
anchors --blink obs--> LOCAL broker --> [collect, rebase, solve, filter] --> CLOUD broker (platform)
                                    \--> uwb/edge/position/<zone> (local viewers)
```

* `tdoa_server/tdoa.py` -- 1:1 port of the firmware's host-tested C (`tdoa_dtu`,
  `tdoa_collect`, `tdoa_solve`, `pos_abg`, the `tdoa_gw.c` flow, `tag_id`).
* `tdoa_server/service.py` -- MQTT glue. Anchor geometry comes from the retained
  `uwb/anchor/setup/<zone>` the gateway publishes (name `ANC-LOBBY-00N` = anchor id N-1).
* `tests/test_tdoa.py` -- unit tests: `python -I -m unittest discover -s tests`
* `tools/xcheck.py` -- compiles the REAL firmware `tdoa_solve.c`/`pos_abg.c` and checks the
  port against them on 4000 random cases (needs gcc; not run in Docker).

## Run

```
cp .env.example .env      # fill the credentials; .env is git-ignored
docker compose up --build -d
docker compose logs -f tdoa-server
```

`PUBLISH_CLOUD=0` keeps everything local (positions only on `uwb/edge/position/<zone>`).

## Node status (field diagnostics without a console)

Each node publishes a retained report on `uwb/anchor/status/<zone>/<id|gw>` every 10 s (reset
cause, uptime, RSSI, the `sync stats` numbers, tau, BLINK counters; the gateway adds CCP
sent/dropped) and an MQTT last-will that turns it into `{"online":0}` if the node vanishes.
The server logs the events that matter (offline, brownout/watchdog reset, reboot, saturated
sync sigma, new re-baselines or rejected outliers) -- see `status_events()` in `service.py`.

## Remote commands (no console needed once mounted)

Signed, single-use, and fail-closed. Provision once on the bench, per node (anchors AND gateway):

```
python -I edge/tools/nodecmd.py genkey          # prints a 64-hex key and the console line
net cmdkey <hex>                                # on each node's console, one time
```

Then, from the machine that has the key (`NODE_CMD_KEY`):

```
python -I edge/tools/nodecmd.py 1 status
python -I edge/tools/nodecmd.py 1 tau_set 312   # persisted, applied live
python -I edge/tools/nodecmd.py 1 tau_mode 1    # 0 legacy, 1 survey
python -I edge/tools/nodecmd.py 1 ant_set 16453 16385   # persisted, applies on the next boot
python -I edge/tools/nodecmd.py 1 sync_reset
python -I edge/tools/nodecmd.py gw reboot       # the gateway only supports status and reboot
```

How it is protected: every command carries HMAC-SHA256 over zone, node, a challenge, the command
and its arguments. The challenge is the random value the node publishes in its status report and
burns on use, so a captured command is worthless and no counter has to survive a reboot. A node with
no key answers `disabled`. See `src/cmd_proto.h` for the format and the threat model.

## Known limits

* **Tid**: the firmware derives it from the tag's EUI through the gateway's seat table.
  The edge sees only the short address, so `TID_MAP=<addr>:<Tid>` maps it; an unmapped tag
  publishes its short address (the gateway's own fallback). A real fix needs the gateway
  to publish its seat table (short address -> EUI) -- not implemented.
* The gateway keeps solving and publishing on the LOCAL broker's
  `uwb/response/position/<zone>`; this service never publishes there.
* `FORWARD_SETUP=1` republishes the retained anchor map to the cloud broker, as the
  gateway did when it was connected to it.
