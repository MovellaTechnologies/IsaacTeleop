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
./xsens_full_body_plugin [collection_id]     # default: xsens_full_body
```

Prove it with a reader on the same collection: an `XsensFullBodySource(name="xsens",
collection_id="xsens_full_body")` in a `TeleopSession` (V1), or MCAP recording on the `full_body`
channel (V2). Per-frame size + FNV-1a fingerprint is logged (~1 Hz) as V3/V4 evidence.

## Not in scope (this is #3863)

The real UDP receiver and its push sink (swapping `convertFrameOutput()` for verified receiver
bytes). Identity/config above are unchanged when that swap happens.
