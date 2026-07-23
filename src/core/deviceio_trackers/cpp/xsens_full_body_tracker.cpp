// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "inc/deviceio_trackers/xsens_full_body_tracker.hpp"

namespace core
{

// ============================================================================
// XsensFullBodyTracker
// ============================================================================

XsensFullBodyTracker::XsensFullBodyTracker(const std::string& collection_id, size_t max_flatbuffer_size)
    : collection_id_(collection_id), max_flatbuffer_size_(max_flatbuffer_size)
{
}

const FullBodyPosePicoTrackedT& XsensFullBodyTracker::get_body_pose(const ITrackerSession& session) const
{
    return static_cast<const IFullBodyTrackerPicoImpl&>(session.get_tracker_impl(*this)).get_body_pose();
}

} // namespace core
