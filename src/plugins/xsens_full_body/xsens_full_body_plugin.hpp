// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <pusherio/schema_pusher.hpp>

// From the MVN checkout (source-only compile input; picofullbody_core, F3 pattern). Produces the
// bare FullBodyPosePico Output payload (784 B) that MVN Studio's live teleop stream puts on the wire.
#include "picofullbody_converter.h"

#include <array>
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
 * @brief T1 spike pusher: builds dummy animated FullBodyPosePico frames with the production
 *        picofullbody::Converter and pushes them verbatim into a distinct Xsens tensor collection
 *        via OpenXR SchemaPusher.
 *
 * Honest Xsens identity (collection_id "xsens_full_body", tensor_identifier "full_body_pose") —
 * this does not pretend to be a Pico device. Paired reader: core::XsensFullBodyTracker on the same
 * collection_id + tensor_identifier (OQ-1). No receiver / UDP / MVN Studio in the loop: frames are
 * generated in-process. The eventual receiver-fed pusher (#3863) swaps the dummy frame builder for
 * verified receiver bytes; identity/config are unchanged.
 */
class XsensFullBodyPlugin
{
public:
    static constexpr size_t MAX_FLATBUFFER_SIZE = 4096; // 784 B payload (T1 F2) + headroom
    static constexpr int NUM_MVN_SEGMENTS = picofullbody::Converter::NUM_MVN_SEGMENTS;

    explicit XsensFullBodyPlugin(const std::string& collection_id);

    //! Build one animated frame and push it verbatim. Called ~250 Hz from main.cpp's loop.
    void update();

private:
    //! Fill segs_ with a finite, unit-quaternion, all-valid pose whose pelvis bobs and one arm
    //! swings as a function of frame, so a downstream reader consuming stale/cached frames is
    //! self-evident (V4). Preserves the converter precondition (finite, near-unit-norm quat).
    void animate(uint64_t frame);

    picofullbody::Converter conv_; //!< NOT thread-safe; one instance, driven from this thread only.
    std::array<picofullbody::SegmentPose, NUM_MVN_SEGMENTS> segs_{};
    uint64_t frame_ = 0;

    std::shared_ptr<core::OpenXRSession> session_;
    core::SchemaPusher pusher_;
};

} // namespace xsens_full_body
} // namespace plugins
