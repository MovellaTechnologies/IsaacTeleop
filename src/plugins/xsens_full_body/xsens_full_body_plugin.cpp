// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "xsens_full_body_plugin.hpp"

#include <oxr/oxr_session.hpp>
#include <oxr_utils/os_time.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace plugins
{
namespace xsens_full_body
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kLoopRateHz = 250.0; // matches main.cpp's push cadence (T1 S5)
constexpr double kPelvisBobHz = 0.5;  // slow, obvious oscillation
constexpr double kArmSwingHz = 0.5;
constexpr int kArmSegment = 8; // an upper-arm segment in XmeSegmentIndex order (dummy liveness only)
constexpr uint64_t kLogEveryFrames = 250; // ~1 s at 250 Hz

// FNV-1a 64-bit, for cheap per-frame payload fingerprinting (V3 verbatim / V4 liveness evidence).
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

XsensFullBodyPlugin::XsensFullBodyPlugin(const std::string& collection_id)
    : session_(std::make_shared<core::OpenXRSession>("XsensFullBodyPusher",
                                                     core::SchemaPusher::get_required_extensions())),
      pusher_(session_->get_handles(),
              core::SchemaPusherConfig{ .collection_id = collection_id,
                                        .max_flatbuffer_size = MAX_FLATBUFFER_SIZE,
                                        .tensor_identifier = "full_body_pose", // MUST match the reader
                                        .localized_name = "Xsens MVN Full Body",
                                        .app_name = "XsensFullBodyPusher" })
{
    // Seed a neutral, per-segment-distinct, all-valid pose (identity orientation) — same well-formed
    // base as tools/teleop_udp/teleop_sender.cpp; animate() perturbs it per frame.
    for (int s = 0; s < NUM_MVN_SEGMENTS; ++s)
    {
        segs_[s].qw = 1.0;
        segs_[s].qx = 0.0;
        segs_[s].qy = 0.0;
        segs_[s].qz = 0.0;
        segs_[s].px = 0.01 * s;
        segs_[s].py = 0.02 * s;
        segs_[s].pz = 0.03 * s;
        segs_[s].valid = true;
    }
}

void XsensFullBodyPlugin::animate(uint64_t frame)
{
    const double t = static_cast<double>(frame) / kLoopRateHz;

    // Pelvis (segment 0) bobs vertically. MVN is Z-up, so pz is height.
    segs_[0].pz = 1.0 + 0.2 * std::sin(2.0 * kPi * kPelvisBobHz * t);

    // Swing one arm: a unit quaternion about Z, angle sweeps +/- ~0.5 rad.
    const double a = 0.5 * std::sin(2.0 * kPi * kArmSwingHz * t);
    segs_[kArmSegment].qw = std::cos(a * 0.5);
    segs_[kArmSegment].qx = 0.0;
    segs_[kArmSegment].qy = 0.0;
    segs_[kArmSegment].qz = std::sin(a * 0.5);
}

void XsensFullBodyPlugin::update()
{
    animate(frame_);

    const std::vector<uint8_t> payload = conv_.convertFrameOutput(segs_); // 784 B, verbatim MVN bytes

    // Spike insurance: attribute any downstream rejection to the Isaac side, not our bytes.
    if (!picofullbody::verifyFullBodyPosePicoPayload(payload.data(), payload.size()))
    {
        throw std::runtime_error("XsensFullBodyPlugin: convertFrameOutput produced an invalid "
                                 "FullBodyPosePico payload (frame " +
                                 std::to_string(frame_) + ")");
    }

    const int64_t now = core::os_monotonic_now_ns();
    // T4 owns real clock mapping; the spike stamps both clocks with the local monotonic now.
    pusher_.push_buffer(payload.data(), payload.size(), now, now);

    // V3/V4 evidence: per-frame size + fingerprint. Distinct hashes prove distinct buffers in
    // (liveness); the constant size re-proves the 784 B max_flatbuffer_size sizing.
    if (frame_ == 0 || frame_ % kLogEveryFrames == 0)
    {
        std::cout << "[XsensFullBodyPusher] frame=" << frame_ << " size=" << payload.size()
                  << " fnv1a64=0x" << std::hex << fnv1a64(payload.data(), payload.size()) << std::dec
                  << " verify=OK" << std::endl;
    }

    ++frame_;
}

} // namespace xsens_full_body
} // namespace plugins
