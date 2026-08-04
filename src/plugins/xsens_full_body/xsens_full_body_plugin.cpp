// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "xsens_full_body_plugin.hpp"

#include <oxr/oxr_session.hpp>
#include <oxr_utils/os_time.hpp>

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace plugins
{
namespace xsens_full_body
{

namespace
{

constexpr uint64_t kLogEveryFrames = 250; // ~1 s at the MVN 60/240 Hz stream / soak cadence

// FNV-1a 64-bit, for cheap per-frame payload fingerprinting (liveness evidence: distinct hashes
// over time prove the pose tracks the recording rather than a frozen/cached frame).
uint64_t fnv1a64(const uint8_t* data, size_t len)
{
    uint64_t h = 1469598103934665603ULL;
    for (size_t i = 0; i < len; ++i)
    {
        h ^= data[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

XsensFullBodyPlugin::XsensFullBodyPlugin(const std::string& collection_id, uint16_t port)
    : port_(port),
      session_(
          std::make_shared<core::OpenXRSession>("XsensFullBodyPusher", core::SchemaPusher::get_required_extensions())),
      pusher_(session_->get_handles(),
              core::SchemaPusherConfig{ .collection_id = collection_id,
                                        .max_flatbuffer_size = MAX_FLATBUFFER_SIZE,
                                        .tensor_identifier = "full_body_pose", // MUST match the reader
                                        .localized_name = "Xsens MVN Full Body",
                                        .app_name = "XsensFullBodyPusher" })
{
}

void XsensFullBodyPlugin::onFrame(const teleop::TeleopFrame& frame)
{
    // Stamp the local common clock with the pusher host's CLOCK_MONOTONIC as late as possible
    // (arrival time at push) and forward the header's raw device time verbatim. The header's
    // sampleTimeNs is a different host's session-relative ms clock and must NOT be forwarded onto
    // the common clock. Sampled right before push_buffer so it does not absorb our own queueing.
    const int64_t localCommonNs = core::os_monotonic_now_ns();

    // push_buffer copies the bytes on its side, so the frame's borrowed payload lifetime is fine.
    pusher_.push_buffer(frame.payload, frame.payloadLen, localCommonNs, frame.rawDeviceTimeNs);

    // First frame of every session (startup or seq reset): header time vs push time.
    if (frame.sessionStart)
    {
        std::cout << "[XsensFullBodyPusher] session seq=" << frame.seq << " header.sampleTimeNs=" << frame.sampleTimeNs
                  << " header.rawDeviceTimeNs=" << frame.rawDeviceTimeNs << " pushed_at_monotonic_ns=" << localCommonNs
                  << std::endl;
    }

    // Periodic liveness/verbatim evidence: constant size re-proves the 784 B max_flatbuffer_size
    // sizing; distinct fnv1a64 across frames proves distinct live bytes (pose is tracking, not frozen).
    if (delivered_ % kLogEveryFrames == 0)
    {
        std::cout << "[XsensFullBodyPusher] delivered=" << delivered_ << " seq=" << frame.seq
                  << " size=" << frame.payloadLen << " fnv1a64=0x" << std::hex
                  << fnv1a64(frame.payload, frame.payloadLen) << std::dec << std::endl;
    }
    ++delivered_;
}

void XsensFullBodyPlugin::run(const std::atomic<bool>& stop)
{
    if (!receiver_.open(port_))
    {
        throw std::runtime_error("XsensFullBodyPlugin: failed to bind UDP port " + std::to_string(port_));
    }
    std::cout << "[XsensFullBodyPusher] listening on 0.0.0.0:" << port_
              << " -> push_buffer (collection tensor full_body_pose)" << std::endl;

    // Blocking: the receiver's sink calls onFrame() -> push_buffer() synchronously per verified
    // frame on this thread. Returns when stop is set or on a fatal socket error.
    receiver_.run([this](const teleop::TeleopFrame& frame) { onFrame(frame); }, stop);

    const teleop::TeleopReceiverStats& s = receiver_.stats();
    std::cout << "[XsensFullBodyPusher] stopped. delivered=" << s.delivered.load() << " resets=" << s.resets.load()
              << " gaps=" << s.gapsTotal.load() << " droppedMalformed=" << s.droppedMalformed.load()
              << " droppedUnverified=" << s.droppedUnverified.load() << " droppedStale=" << s.droppedStale.load()
              << " droppedTruncated=" << s.droppedTruncated.load() << " warnedNonWholeMs=" << s.warnedNonWholeMs.load()
              << " warnedTimeBackward=" << s.warnedTimeBackward.load() << std::endl;
}

} // namespace xsens_full_body
} // namespace plugins
