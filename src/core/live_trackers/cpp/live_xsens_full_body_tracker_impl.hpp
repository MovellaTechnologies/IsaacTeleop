// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "inc/live_trackers/schema_tracker.hpp"

#include <deviceio_trackers/xsens_full_body_tracker.hpp>
#include <oxr_utils/oxr_session_handles.hpp>
#include <schema/full_body_generated.h>

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace core
{

// NB: the native pico impl header (live_full_body_tracker_pico_impl.hpp) already defines
// `FullBodyMcapChannels` as this exact type. Use a distinct alias here so both headers can coexist
// in one translation unit (the factory includes both).
using XsensFullBodyMcapChannels = McapTrackerChannels<FullBodyPosePicoRecord, FullBodyPosePico>;
using FullBodySchemaTracker = SchemaTracker<FullBodyPosePicoRecord, FullBodyPosePico>;

// Reader-side full body tracker impl: decodes FullBodyPosePico frames from a tensor collection
// pushed by the Xsens add_device pusher. Mechanical mirror of LiveJointStateTrackerImpl — the only
// real content is one SchemaTracker<FullBodyPosePicoRecord, FullBodyPosePico> and its tensor config.
// Contrast LiveFullBodyTrackerPicoImpl, which reads OpenXR body joints natively (no collection).
class LiveXsensFullBodyTrackerImpl : public IFullBodyTrackerPicoImpl
{
public:
    static std::vector<std::string> required_extensions()
    {
        return SchemaTrackerBase::get_required_extensions();
    }
    static std::unique_ptr<XsensFullBodyMcapChannels> create_mcap_channels(mcap::McapWriter& writer,
                                                                           std::string_view base_name);

    LiveXsensFullBodyTrackerImpl(const OpenXRSessionHandles& handles,
                                 const XsensFullBodyTracker* tracker,
                                 std::unique_ptr<XsensFullBodyMcapChannels> mcap_channels);

    LiveXsensFullBodyTrackerImpl(const LiveXsensFullBodyTrackerImpl&) = delete;
    LiveXsensFullBodyTrackerImpl& operator=(const LiveXsensFullBodyTrackerImpl&) = delete;
    LiveXsensFullBodyTrackerImpl(LiveXsensFullBodyTrackerImpl&&) = delete;
    LiveXsensFullBodyTrackerImpl& operator=(LiveXsensFullBodyTrackerImpl&&) = delete;

    void update(int64_t monotonic_time_ns) override;
    const FullBodyPosePicoTrackedT& get_body_pose() const override;

private:
    std::unique_ptr<XsensFullBodyMcapChannels> mcap_channels_;
    FullBodySchemaTracker m_schema_reader;
    FullBodyPosePicoTrackedT m_tracked;
};

} // namespace core
