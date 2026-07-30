<!-- SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Xsens Full Body Pusher

An `add_device` pusher that receives the live Xsens full-body stream from MVN Studio over UDP and
publishes it (`FullBodyPosePico`, 24 joints) into a **distinct Xsens tensor collection**, consumed by
`core::XsensFullBodyTracker` + `XsensFullBodySource`.

Receiver and pusher live in **one process**: the UDP receive loop hands each verified frame straight
to `SchemaPusher::push_buffer` on the same thread — no localhost hop, no second framing, no
re-serialization. MVN Studio has already converted the pose, so nothing here interprets Xsens data;
the payload bytes are forwarded verbatim.

## Why a distinct collection (not the existing `full_body`)

The stock `FullBodyTrackerPico` reads OpenXR body joints natively via `xrLocateBodyJointsBD` — there
is **no tensor collection** behind `full_body` for a pusher to target. Full-body data reaches the
retargeting engine over the pusher route **only** via a tensor-collection reader
(`SchemaTracker<FullBodyPosePicoRecord, FullBodyPosePico>`), which requires a distinct
`collection_id` / `tensor_identifier`. This is the established add_device pattern (pedal / so101 /
oak). Honest Xsens identity: we do not pretend to be a Pico device.

## Identity / config (pusher `SchemaPusherConfig` ↔ reader `SchemaTrackerConfig` — must agree)

| Field | Value |
|-|-|
| `collection_id` | `"xsens_full_body"` (CLI arg, this as default) |
| `tensor_identifier` | `"full_body_pose"` (fixed constant, hardcoded identically on both sides) |
| `max_flatbuffer_size` | `4096` (payload is a constant 784 B; >5× headroom) |
| `localized_name` | `"Xsens MVN Full Body"` |
| `app_name` | `"XsensFullBodyPusher"` |
| extensions | `SchemaPusher::get_required_extensions()` → `XR_NVX1_push_tensor` + `XR_NVX1_tensor_data` + time-conversion (**no** body-tracking ext) |

A `collection_id` mismatch between pusher and reader is **silent no-data**, not an error — pass the
same value to both.

## Frame source: the live teleop wire

`teleop_receiver.{h,cpp}` binds UDP (default 9764) and, per datagram:

1. deserializes the fixed 36-byte header (magic, version, seq, sample time, raw device time,
   payload length);
2. calls `verifyFullBodyPosePicoPayload()` **before** any FlatBuffer access — `GetRoot` does no
   bounds checking, so this gate is what makes a malformed datagram safe;
3. hands the payload pointer + length and the header times to the sink, which pushes them verbatim.

Stream semantics: a `seq` reset to 0 is a **session boundary** (MVN Studio restarted — the pusher
keeps running and picks up the new session), a `seq` gap is UDP loss and is counted, and a malformed
or unverifiable datagram is dropped. Nothing in that set is fatal.

The framing and verify gate are compiled from the MVN sources (`picofullbody_core`), so both ends of
the wire are built from one definition.

## Timestamps

```
sample_time_local_common_clock_ns = CLOCK_MONOTONIC on this host, sampled at the push call
sample_time_raw_device_clock_ns   = the header's raw device time, forwarded verbatim
```

The header's sample time is a *different* host's session-relative millisecond clock, so it cannot be
placed on the local common clock — restamping at push is what keeps the pushed clock monotonic even
when the sending side rewinds (recording playback loops), and it matches how the pedal sample stamps
at read.

## Build

**In-tree (CMake).** Built automatically as part of the IsaacTeleop super-build when the sibling MVN
checkout is present (`<trunk>/{IsaacTeleop,mvn,3p,linux-x64}`). Override paths with
`-DXSENS_MVN_PICOFULLBODY_DIR=...`, `-DXSENS_3P_FLATBUFFERS_INCLUDE=...`, `-DXSENS_XLIB_INCLUDE=...`.
If the MVN sources are not found the plugin is **skipped** with a status message and never fails the
main build — which also means CI, where no MVN checkout exists, does not compile it.

**Standalone (fast iteration).** `./build.sh` compiles against the IsaacTeleop build-tree static libs
(`cmake --build build` must have been run once) + the MVN sources. Produces the same executable
without reconfiguring the super-build.

## Run

Requires the CloudXR OpenXR runtime with `XR_NVX1_push_tensor` + `XR_NVX1_tensor_data` (see
`deps/cloudxr/`, `examples/oxr/README.md` for the `XR_RUNTIME_JSON` export). Launch it the same way
the pedal sample launches.

```bash
./xsens_full_body_plugin [collection_id] [udp_port]   # defaults: xsens_full_body, 9764
```

Start the pusher **before** any reader — it creates the collection. Expect:

```
Xsens Full Body Pusher (collection: xsens_full_body, tensor: full_body_pose, udp: 0.0.0.0:9764)
[XsensFullBodyPusher] listening on 0.0.0.0:9764 -> push_buffer (collection tensor full_body_pose)
```

Then feed it, either from MVN Studio (Network Streamer ▸ **Isaac Teleop** preset ▸ `127.0.0.1:9764`
▸ Play) or headless with `mvn_isaac_devtools/tools/teleop_udp/teleop_sender 6000 9764 1`.

Per session (startup or `seq` reset) it logs the header times next to the push-time monotonic stamp;
roughly once a second it logs `delivered=… seq=… size=… fnv1a64=…` — a constant `size` re-confirms
the `max_flatbuffer_size` sizing and changing fingerprints prove live bytes rather than a frozen or
cached pose. `Ctrl-C` (SIGINT/SIGTERM) stops it cleanly and prints the final counters: `delivered`,
`resets`, `gaps`, the four `dropped*` reasons, and the two `warned*` reasons.

Confirm the data lands by reading the same collection: `examples/xsens_full_body/xsens_full_body_printer`
(C++), an `XsensFullBodySource(name="xsens", collection_id="xsens_full_body")` in a `TeleopSession`,
or MCAP recording on the `full_body` channel.

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
`cmake --install build/examples/xsens_full_body` after a rebuild.

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

## Behaviour and limitations

- **Single-threaded and blocking.** `run()` owns the thread: receive → verify → push, synchronously
  per frame.
- **One collection and one port per instance.** To serve several readers, point them all at the same
  collection rather than starting a second pusher.
- **The OpenXR session is established once, at construction.** If the CloudXR runtime restarts, the
  pusher does not re-establish its session — restart the plugin.
- **A fatal socket error ends the run.** Unexpected `recvfrom` errors stop the loop and print the
  final counters rather than retrying; the receive timeout itself (idle stream) is not an error.
- **Frame loss is visible, not repaired.** UDP has no retransmission by design; `gaps` and the
  `dropped*` counters are the record of what did not arrive intact.
