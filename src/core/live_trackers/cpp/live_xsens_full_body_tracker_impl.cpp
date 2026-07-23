// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "live_xsens_full_body_tracker_impl.hpp"

#include <mcap/recording_traits.hpp>
#include <schema/full_body_bfbs_generated.h>

namespace core
{

namespace
{

SchemaTrackerConfig make_xsens_full_body_tensor_config(const XsensFullBodyTracker* tracker)
{
    SchemaTrackerConfig cfg;
    cfg.collection_id = tracker->collection_id();
    cfg.max_flatbuffer_size = tracker->max_flatbuffer_size();
    cfg.tensor_identifier = "full_body_pose"; // MUST equal the pusher's tensor_identifier
    cfg.localized_name = "XsensFullBodyTracker";
    return cfg;
}

} // namespace

// ============================================================================
// LiveXsensFullBodyTrackerImpl
// ============================================================================

std::unique_ptr<XsensFullBodyMcapChannels> LiveXsensFullBodyTrackerImpl::create_mcap_channels(mcap::McapWriter& writer,
                                                                                              std::string_view base_name)
{
    return std::make_unique<XsensFullBodyMcapChannels>(
        writer, base_name, FullBodyPicoRecordingTraits::schema_name,
        std::vector<std::string>(FullBodyPicoRecordingTraits::recording_channels.begin(),
                                 FullBodyPicoRecordingTraits::recording_channels.end()));
}

LiveXsensFullBodyTrackerImpl::LiveXsensFullBodyTrackerImpl(const OpenXRSessionHandles& handles,
                                                           const XsensFullBodyTracker* tracker,
                                                           std::unique_ptr<XsensFullBodyMcapChannels> mcap_channels)
    : mcap_channels_(std::move(mcap_channels)),
      m_schema_reader(handles,
                      make_xsens_full_body_tensor_config(tracker),
                      mcap_channels_.get(),
                      /*mcap_channel_index=*/0)
{
    // FullBodyPicoRecordingTraits declares a single "full_body" channel (no separate _tracked
    // channel), so there is no mcap_channel_tracked_index — the default std::nullopt applies.
}

void LiveXsensFullBodyTrackerImpl::update(int64_t /*monotonic_time_ns*/)
{
    // Policy: SchemaTracker throws on critical OpenXR/tensor API failures.
    // Missing collection/no new data are treated as common non-fatal cases.
    m_schema_reader.update(m_tracked.data);
}

const FullBodyPosePicoTrackedT& LiveXsensFullBodyTrackerImpl::get_body_pose() const
{
    return m_tracked;
}

} // namespace core
