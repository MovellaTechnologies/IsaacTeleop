# SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Xsens Full Body Source Node - DeviceIO to Retargeting Engine converter.

Converts raw FullBodyPosePicoT flatbuffer data (pushed by the Xsens add_device pusher into a
distinct tensor collection and decoded by XsensFullBodyTracker) to the standard FullBodyInput
tensor format.

This is the tensor-collection sibling of ``FullBodySource``: ``FullBodySource`` wraps the native
``FullBodyTrackerPico`` (OpenXR ``xrLocateBodyJointsBD``, no collection), while this node wraps
``XsensFullBodyTracker`` (a ``SchemaTracker`` reader on ``collection_id``). The ``input_spec`` /
``output_spec`` / ``_compute_fn`` are identical — the ``FullBodyPosePicoT -> FullBodyInput``
conversion is reused verbatim, so downstream retargeting is unchanged.
"""

import numpy as np
from typing import Any, TYPE_CHECKING
from .interface import IDeviceIOSource
from ..interface.retargeter_core_types import (
    RetargeterIO,
    RetargeterIOType,
)
from ..interface.tensor_group import TensorGroup
from ..tensor_types import FullBodyInput, FullBodyInputIndex
from ..interface.tensor_group_type import OptionalType
from ..tensor_types.standard_types import NUM_BODY_JOINTS_PICO
from .deviceio_tensor_types import DeviceIOFullBodyPosePicoTracked

if TYPE_CHECKING:
    from isaacteleop.deviceio import ITracker
    from isaacteleop.schema import FullBodyPosePicoT, FullBodyPosePicoTrackedT


class XsensFullBodySource(IDeviceIOSource):
    """
    Stateless converter: DeviceIO FullBodyPosePicoT (Xsens pusher) -> FullBodyInput tensors.

    Inputs:
        - "deviceio_full_body": Raw FullBodyPosePicoT flatbuffer

    Outputs (Optional — absent when no sample / the collection is unavailable):
        - "full_body": OptionalTensorGroup (check ``.is_none`` before access)

    Usage:
        source = XsensFullBodySource(name="xsens", collection_id="xsens_full_body")
        # In a TeleopSession the tracker is discovered from the pipeline and polled each frame.
    """

    FULL_BODY = "full_body"

    def __init__(self, name: str, collection_id: str = "xsens_full_body") -> None:
        """Initialize the Xsens full body source node.

        Creates an XsensFullBodyTracker instance for TeleopSession to discover and use.

        Args:
            name: Unique name for this source node.
            collection_id: Tensor collection ID for the Xsens pusher (must match the plugin / pusher).
        """
        import isaacteleop.deviceio as deviceio

        self._body_tracker = deviceio.XsensFullBodyTracker(collection_id)
        self._collection_id = collection_id
        super().__init__(name)

    def get_tracker(self) -> "ITracker":
        """Get the XsensFullBodyTracker instance.

        Returns:
            The XsensFullBodyTracker instance for TeleopSession to initialize
        """
        return self._body_tracker

    def poll_tracker(self, deviceio_session: Any) -> RetargeterIO:
        """Poll body tracker and return input data.

        Args:
            deviceio_session: The active DeviceIO session.

        Returns:
            Dict with "deviceio_full_body" TensorGroup containing raw
            FullBodyPosePicoT data.
        """
        body_pose = self._body_tracker.get_body_pose(deviceio_session)
        source_inputs = self.input_spec()
        result: RetargeterIO = {}
        for input_name, group_type in source_inputs.items():
            tg = TensorGroup(group_type)
            tg[0] = body_pose
            result[input_name] = tg
        return result

    def input_spec(self) -> RetargeterIOType:
        """Declare DeviceIO full body input."""
        return {
            "deviceio_full_body": DeviceIOFullBodyPosePicoTracked(),
        }

    def output_spec(self) -> RetargeterIOType:
        """Declare standard full body output (Optional — may be absent)."""
        return {
            "full_body": OptionalType(FullBodyInput()),
        }

    def _compute_fn(self, inputs: RetargeterIO, outputs: RetargeterIO, context) -> None:
        """
        Convert DeviceIO FullBodyPosePicoT to standard FullBodyInput tensors.

        Calls ``set_none()`` on the output when no sample is available.

        Args:
            inputs: Dict with "deviceio_full_body" containing FullBodyPosePicoTrackedT wrapper
            outputs: Dict with "full_body" OptionalTensorGroup
            context: Shared ComputeContext for the current step (carries GraphTime).
        """
        tracked: "FullBodyPosePicoTrackedT" = inputs["deviceio_full_body"][0]
        body_pose: "FullBodyPosePicoT | None" = tracked.data

        if body_pose is None:
            outputs["full_body"].set_none()
            return

        group = outputs["full_body"]

        positions = np.zeros((NUM_BODY_JOINTS_PICO, 3), dtype=np.float32)
        orientations = np.zeros((NUM_BODY_JOINTS_PICO, 4), dtype=np.float32)
        valid = np.zeros(NUM_BODY_JOINTS_PICO, dtype=np.uint8)

        if body_pose.joints is not None:
            for i in range(NUM_BODY_JOINTS_PICO):
                joint = body_pose.joints.joints(i)
                positions[i] = [
                    joint.pose.position.x,
                    joint.pose.position.y,
                    joint.pose.position.z,
                ]
                orientations[i] = [
                    joint.pose.orientation.x,
                    joint.pose.orientation.y,
                    joint.pose.orientation.z,
                    joint.pose.orientation.w,
                ]
                valid[i] = 1 if joint.is_valid else 0

        group[FullBodyInputIndex.JOINT_POSITIONS] = positions
        group[FullBodyInputIndex.JOINT_ORIENTATIONS] = orientations
        group[FullBodyInputIndex.JOINT_VALID] = valid
