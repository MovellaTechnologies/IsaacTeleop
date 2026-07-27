# SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Visualize the LIVE Xsens full-body pose in viser (milestone step 3a).

Xsens variant of live_full_body.py: instead of the native FullBodyTrackerPico (a real PICO
headset over CloudXR), this runs XsensFullBodySource on the `xsens_full_body` tensor collection
that our add_device pusher (src/plugins/xsens_full_body) fills from the MVN Studio teleop wire.

The pose reaches viser as a live PICO-joint skeleton (green=valid, red=lost), Y-up, ~0.96 m
standing. This is the "see it move" milestone that IS supported in this repo. A robot in Isaac
Sim is a separate, larger effort (no full-body->humanoid retargeter exists here, and Isaac Sim
lives in the external Isaac Lab repo) — see the guide.

RUNTIME STACK (three things must already be up; see the guide's "run it" section):
  1. CloudXR:  cd ~/dev_trunk/IsaacTeleop && ./scripts/run_cloudxr.sh &
  2. Pusher:   source ~/.cloudxr/run/cloudxr.env &&
               ~/dev_trunk/IsaacTeleop/build/src/plugins/xsens_full_body/xsens_full_body_plugin &
  3. Frames:   MVN Studio "Isaac Teleop" preset streaming to :9764, OR headless:
               ~/dev_trunk/mvn_isaac_devtools/tools/teleop_udp/teleop_sender 6000 9764 1

Then, WITH cloudxr.env sourced in THIS shell (TeleopSession opens its own OpenXR reader session):
    cd ~/dev_trunk/IsaacTeleop/examples/mcap_record_replay/python
    python xsens_live_full_body.py [--port 8080] [--collection xsens_full_body]
Open the printed viser URL (http://localhost:8080) in a browser. Ctrl+C to stop.
"""

import argparse
import sys
import time

import numpy as np
import viser

from isaacteleop.retargeting_engine.deviceio_source_nodes import XsensFullBodySource
from isaacteleop.retargeting_engine.interface import OutputCombiner
from isaacteleop.retargeting_engine.tensor_types.indices import FullBodyInputIndex
from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig

from common import BODY_JOINT_NAMES, FullBodyViz


def build_xsens_full_body_pipeline(collection_id: str):
    """Full-body-only pipeline fed by the Xsens tensor collection (no controllers/headset)."""
    src = XsensFullBodySource(name="full_body", collection_id=collection_id)
    return OutputCombiner({"full_body": src.output(XsensFullBodySource.FULL_BODY)})


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Viser bind address")
    parser.add_argument("--port", type=int, default=8080, help="Viser HTTP port")
    parser.add_argument("--collection", default="xsens_full_body", help="Tensor collection id (match the pusher)")
    args = parser.parse_args(argv[1:])

    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.set_up_direction("+y")  # Pico full-body is Y-up
    server.scene.add_grid(name="/grid", width=2.0, height=2.0, cell_size=0.1)

    config = TeleopSessionConfig(
        app_name="XsensLiveFullBody",
        pipeline=build_xsens_full_body_pipeline(args.collection),
    )

    # NOTE: no CloudXRLauncher here. Start CloudXR + the pusher yourself (see module docstring);
    # TeleopSession opens its OWN OpenXR reader session in __enter__, so this shell must have
    # `source ~/.cloudxr/run/cloudxr.env` applied. The pose arrives via the pusher, not a headset.
    with TeleopSession(config) as session:
        viz = FullBodyViz(server)
        print(f"[xsens-live] viser at http://localhost:{args.port}  (collection={args.collection})")
        print("[xsens-live] waiting for frames on the collection… (Ctrl+C to stop)")
        try:
            while True:
                result = session.step()
                full_body = result["full_body"]
                if full_body.is_none:
                    viz.update(None, None)
                    n_valid = 0
                else:
                    positions = np.asarray(full_body[FullBodyInputIndex.JOINT_POSITIONS], dtype=np.float32)
                    valid = np.asarray(full_body[FullBodyInputIndex.JOINT_VALID], dtype=np.uint8)
                    viz.update(positions, valid)
                    n_valid = int(np.count_nonzero(valid))
                if session.frame_count % 60 == 0:
                    print(f"[xsens-live] frame={session.frame_count}  joints={n_valid:02d}/{len(BODY_JOINT_NAMES)}")
                time.sleep(1 / 60)
        except KeyboardInterrupt:
            pass

    print("[xsens-live] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
