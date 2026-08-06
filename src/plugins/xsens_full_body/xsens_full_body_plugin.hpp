// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "teleop_receiver.h"

#include <pusherio/schema_pusher.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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

    //! Sized for an operator-driven CloudXR restart rather than a transient blip: ~23 s total.
    static constexpr int MAX_SESSION_RECOVERY_ATTEMPTS = 8;

private:
    //! Receiver sink: restamp local-common clock at push, forward raw device time, push verbatim.
    //! Runs on the receive thread; the payload is valid only for this call (push_buffer copies).
    void onFrame(const teleop::TeleopFrame& frame);

    //! Create (or re-create) the session and the SchemaPusher bound to its handles. May throw.
    void establishSession();

    //! Re-establish after a failed push, bounded retries with backoff. False if the budget ran out
    //! or \c stop_ was set. Blocks receiving: without a pusher, arriving frames have nowhere to go.
    bool recoverSession();

    //! Printed on the normal stop path AND when a fatal push unwinds, so a soak keeps its counters.
    void printStats() const;

    uint16_t port_;
    std::string collectionId_; //!< kept so the pusher can be rebuilt after a session loss
    uint64_t delivered_ = 0; //!< receive-thread-only frame counter for periodic evidence logging
    uint64_t sessionRecoveries_ = 0; //!< successful re-establishes after a push failure
    uint64_t pushFailures_ = 0; //!< push_buffer throws seen (each opens a recovery episode)

    //! Borrowed for the duration of run(), so recoverSession()'s backoff stays interruptible.
    const std::atomic<bool>* stop_ = nullptr;

    teleop::TeleopUdpReceiver receiver_;

    std::shared_ptr<core::OpenXRSession> session_;
    //! optional because SchemaPusher has deleted copy AND move: a recovery must construct in place.
    std::optional<core::SchemaPusher> pusher_;
};

} // namespace xsens_full_body
} // namespace plugins
