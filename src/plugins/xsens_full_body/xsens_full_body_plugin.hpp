// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "teleop_receiver.h"

#include <pusherio/schema_pusher.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace core
{
class OpenXRSession;
}

namespace plugins
{
namespace xsens_full_body
{

/*!
 * @brief Live pusher: binds the embedded UDP teleop receiver to OpenXR SchemaPusher so real MVN
 *        Studio "Isaac Teleop" frames flow into a distinct Xsens tensor collection.
 *
 * Topology: IN-TREE, ONE process. The receiver runs its blocking receive loop on this thread; its
 * sink fires synchronously per verified frame and calls push_buffer directly (no localhost hop, no
 * second framing). SchemaPusher is driven only from that one (receive) thread.
 *
 * Xsens identity (collection_id "xsens_full_body", tensor_identifier "full_body_pose") — this does
 * not pretend to be a Pico device. Paired reader: core::XsensFullBodyTracker on the same
 * collection_id + tensor_identifier. Conversion happens MVN-side (license-gated); the wire carries
 * already-full-body bytes, so deserialize+verify here is parsing, not conversion.
 *
 * Timestamp mapping: the sink stamps sample_time_local_common_clock_ns with the pusher host's
 * CLOCK_MONOTONIC at push (arrival time) and forwards the header's rawDeviceTimeNs verbatim as the
 * raw device clock.
 */
class XsensFullBodyPlugin
{
public:
    static constexpr size_t MAX_FLATBUFFER_SIZE = 4096; // 784 B payload + headroom
    static constexpr uint16_t DEFAULT_PORT = 9764; // MVN Studio Isaac Teleop wire default

    XsensFullBodyPlugin(const std::string& collection_id, uint16_t port);

    //! Bind the receiver and run its blocking receive loop on THIS thread. For each verified frame
    //! the sink pushes verbatim bytes via push_buffer. Returns when \c stop becomes true or on a
    //! fatal socket error. \throws std::runtime_error if the UDP port cannot be bound.
    void run(const std::atomic<bool>& stop);

    const teleop::TeleopReceiverStats& stats() const
    {
        return receiver_.stats();
    }

private:
    //! Receiver sink: restamp local-common clock at push, forward raw device time, push verbatim.
    //! Runs on the receive thread; the payload is valid only for this call (push_buffer copies).
    void onFrame(const teleop::TeleopFrame& frame);

    uint16_t port_;
    uint64_t delivered_ = 0; //!< receive-thread-only frame counter for periodic evidence logging

    teleop::TeleopUdpReceiver receiver_;

    std::shared_ptr<core::OpenXRSession> session_;
    core::SchemaPusher pusher_;
};

} // namespace xsens_full_body
} // namespace plugins
