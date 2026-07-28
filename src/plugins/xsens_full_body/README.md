<!-- SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Xsens Full Body Pusher (T1 spike)

An `add_device` pusher that streams **full-body pose** (`FullBodyPosePico`, 24 joints) into a
**distinct Xsens tensor collection**, consumed by the fork-side reader
`core::XsensFullBodyTracker` + `XsensFullBodySource`. This is the T1 spike (ADO **#3862**): it
generates **dummy animated frames in-process** (no receiver, no UDP, no MVN Studio) to prove the
push → tensor collection → reader decode mechanism end to end.

## Why a distinct collection (not the existing `full_body`)

The stock `FullBodyTrackerPico` reads OpenXR body joints natively via `xrLocateBodyJointsBD` — there
is **no tensor collection** behind `full_body` for a pusher to target. Full-body data reaches the
retargeting engine over the pusher route **only** via a tensor-collection reader
(`SchemaTracker<FullBodyPosePicoRecord, FullBodyPosePico>`), which requires a distinct
`collection_id` / `tensor_identifier`. This is the established add_device pattern (pedal / so101 /
oak). Honest Xsens identity: we do not pretend to be a Pico device. See
`mvn_isaac_devtools/docs/plans/feature2_pusher_receiver_tasks/OQ1_push_identity_findings.md`.

## Identity / config (pusher `SchemaPusherConfig` ↔ reader `SchemaTrackerConfig` — must agree)

| Field | Value |
|-|-|
| `collection_id` | `"xsens_full_body"` (CLI arg, this as default) |
| `tensor_identifier` | `"full_body_pose"` (fixed constant, hardcoded identically on both sides) |
| `max_flatbuffer_size` | `4096` (payload is a constant 784 B; >5× headroom) |
| `localized_name` | `"Xsens MVN Full Body"` |
| `app_name` | `"XsensFullBodyPusher"` |
| extensions | `SchemaPusher::get_required_extensions()` → `XR_NVX1_push_tensor` + `XR_NVX1_tensor_data` + time-conversion (**no** body-tracking ext) |

## Dummy frame builder

The payload is built by the **production** `picofullbody::Converter::convertFrameOutput()` (from the
MVN checkout, `mvn_studio/src/picofullbody_core/`) — the exact bytes MVN Studio's live teleop stream
puts on the wire (F1/F3). The pose is animated (pelvis bob + one arm swing, unit quaternions) so a
reader consuming stale/cached frames is self-evident. Each frame is gated with
`verifyFullBodyPosePicoPayload()` so any downstream rejection is attributable to the Isaac side, not
our bytes. Bytes are handed to `push_buffer` verbatim (no re-serialization).

## Build

**In-tree (CMake, eventual home).** Built automatically as part of the IsaacTeleop super-build when
the sibling MVN checkout is present (`<trunk>/{IsaacTeleop,mvn,3p,linux-x64}`). Override paths with
`-DXSENS_MVN_PICOFULLBODY_DIR=...`, `-DXSENS_3P_FLATBUFFERS_INCLUDE=...`, `-DXSENS_XLIB_INCLUDE=...`.
If the MVN converter is not found the plugin is skipped (never fails the main build).

**Standalone (fast T1 iteration).** `./build.sh` compiles against the IsaacTeleop build-tree static
libs (`cmake --build build` must have been run once) + the MVN converter. Produces the same
executable without reconfiguring the super-build.

## Run

Requires the CloudXR OpenXR runtime with `XR_NVX1_push_tensor` + `XR_NVX1_tensor_data` (see
`deps/cloudxr/`, `examples/oxr/README.md` for the `XR_RUNTIME_JSON` export). Launch it the same way
the pedal sample launches.

```bash
./xsens_full_body_plugin [collection_id] [udp_port]   # defaults: xsens_full_body, 9764
```

### Or launch the whole session with the rig launcher

`rigs/xsens_full_body.yaml` starts the pusher, the C++ printer and the viser skeleton as three
tmux panes, each waiting for the CloudXR runtime and sourcing its env automatically:

```bash
cd <IsaacTeleop>
python -m isaacteleop.rig rigs/xsens_full_body.yaml                # let the rig manage the runtime
python -m isaacteleop.rig rigs/xsens_full_body.yaml --no-runtime   # only when a runtime is ALREADY up
python -m isaacteleop.rig rigs/xsens_full_body.yaml --kill         # tear the session down
```

`--no-runtime` skips the runtime *pane*, not the runtime *wait*: every worker pane still blocks up
to 120 s on `~/.cloudxr/run/runtime_started` and auto-runs only once `cloudxr.env` is sourced. With
no runtime up, all panes therefore time out with `[cloudxr] runtime not ready after 120s` and drop
to a shell with the command pre-typed. Check with `kill -0 $(cat ~/.cloudxr/run/cloudxr.pid)` first,
and prefer the managed form above when in doubt. Note a re-run *reattaches* to an existing session
(reattach is decided before preflight), so apply YAML edits with `--kill` then relaunch.

Run it from the interpreter you want in the panes — `{python}` in the rig expands to
`sys.executable`, and tmux panes do not inherit an activated venv. The rig references the binaries
under `install/`, so refresh them with
`cmake --install build/src/plugins/xsens_full_body` and
`cmake --install build/examples/xsens_full_body` after a rebuild. Feed it from MVN Studio
(Network Streamer ▸ **Isaac Teleop** preset ▸ 127.0.0.1:9764 ▸ Play) or headless with
`mvn_isaac_devtools/tools/teleop_udp/teleop_sender 6000 9764 1`.

#### Closing the session

`Ctrl-b d` only **detaches** — every pane and the runtime keep running. To actually shut down, stop
the runtime *first*, then destroy the session:

1. `Ctrl-C` in the runtime pane (`Ctrl-C` kills the pane's command, not the pane).
2. `python -m isaacteleop.rig rigs/xsens_full_body.yaml --kill`.

That order matters. `--kill` is only `tmux kill-session`, and tmux destroys panes with `SIGHUP`,
whereas the runtime handles `SIGINT`/`SIGTERM` and registers its cleanup through `atexit` — none of
which runs on a `SIGHUP` death. Its cleanup is what removes `runtime_started` and `ipc_cloudxr`.

This is not hypothetical — measured on Linux, a bare `--kill` with no preceding `Ctrl-C` leaves both
`runtime_started` and `ipc_cloudxr` in place **and** the runtime process alive, reparented to
`systemd --user` as its own session leader (`pid == pgid == sid`). It is started with
`start_new_session`, so the `SIGHUP` tmux sends to the pane's process group never reaches it. Verify:

```bash
tmux ls                                       # no xsens_full_body session
ls ~/.cloudxr/run/                            # no runtime_started, no ipc_cloudxr
kill -0 $(cat ~/.cloudxr/run/cloudxr.pid)     # should fail
ps -eo pid,cmd | grep -Ei "cloudxr|monado"    # no survivors
```

A leftover `cloudxr.pid` holding a dead pid is normal and harmless — the runtime does not remove its
own pid file, and the next *managed* launch calls `_cleanup_stale_runtime`, which clears
`ipc_cloudxr`, `runtime_started`, `monado.pid` and `cloudxr.pid` before starting. The combination to
avoid is a stale `runtime_started` plus `--no-runtime`: panes would source the env and auto-run
against a dead runtime instead of failing honestly on the 120 s timeout. Manual reset:

```bash
kill $(cat ~/.cloudxr/run/cloudxr.pid) 2>/dev/null
rm -f ~/.cloudxr/run/{runtime_started,ipc_cloudxr,cloudxr.pid,monado.pid}
```

If the `Ctrl-b` prefix appears to do nothing, the terminal is probably eating it — VS Code binds
`Ctrl-B` to *Toggle Primary Sidebar*, so it never reaches tmux. Use a standalone terminal, or drive
the session from a second shell (`tmux select-pane -t xsens_full_body.1`,
`tmux send-keys -t xsens_full_body.0 C-c`).

Prove it with a reader on the same collection: an `XsensFullBodySource(name="xsens",
collection_id="xsens_full_body")` in a `TeleopSession` (V1), or MCAP recording on the `full_body`
channel (V2). Per-frame size + FNV-1a fingerprint is logged (~1 Hz) as V3/V4 evidence.

## Not in scope (this is #3863)

The real UDP receiver and its push sink (swapping `convertFrameOutput()` for verified receiver
bytes). Identity/config above are unchanged when that swap happens.
