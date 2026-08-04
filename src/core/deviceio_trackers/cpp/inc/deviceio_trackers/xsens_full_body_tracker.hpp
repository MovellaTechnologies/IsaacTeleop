// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <deviceio_base/full_body_tracker_pico_base.hpp>
#include <schema/full_body_generated.h>

#include <cstddef>
#include <string>

namespace core
{

/*!
 * @brief Reader-side full body tracker: consumes ``FullBodyPosePico`` bytes from a tensor
 *        collection pushed by a third-party producer (the Xsens add_device pusher).
 *
 * Distinct from ``FullBodyTrackerPico``, which reads OpenXR body joints natively via
 * ``xrLocateBodyJointsBD`` and never touches a tensor collection. This tracker sits on the
 * tensor-collection path (like ``JointStateTracker`` / ``Generic3AxisPedalTracker``): a
 * ``SchemaPusher`` pushes serialized ``FullBodyPosePico`` frames into a named collection and
 * this reader decodes them. Pusher and reader interoperate only when their ``collection_id``,
 * ``tensor_identifier`` (``"full_body_pose"``), ``max_flatbuffer_size``, and FlatBuffer schema
 * all agree.
 *
 * The payload is the already-full-body 24-joint pose (produced MVN-side by picofullbody::Converter
 * and forwarded verbatim); this reader parses it, it does not convert Xsens data. Downstream the
 * decoded ``FullBodyPosePicoTrackedT`` feeds the standard ``FullBodyInput`` retargeting output,
 * unchanged from the native path.
 *
 * After each ``ITrackerSession::update()`` that includes this tracker, ``get_body_pose(session)``
 * reflects the implementation's tracked snapshot. As with other ``SchemaTracker``-backed trackers,
 * the live backend may retain the last-known sample when a tick has no new samples while the
 * collection remains available (``data`` stays non-null but may be stale); ``data`` is null only
 * when no sample has arrived yet or the collection is unavailable.
 *
 * Usage:
 * @code
 * auto tracker = std::make_shared<XsensFullBodyTracker>("xsens_full_body");
 * // ... register the tracker with a session, then each tick: ...
 * session->update();
 * const auto& tracked = tracker->get_body_pose(*session);
 * @endcode
 */
class XsensFullBodyTracker : public ITracker
{
public:
    //! Default maximum FlatBuffer size for FullBodyPosePico messages.
    //! The payload is a constant 784 B for the fixed 24-joint layout; 4096 leaves >5x
    //! headroom for schema drift while staying trivially small. Pusher and tracker must agree on
    //! this value (it sizes the fixed tensor buffer).
    static constexpr size_t DEFAULT_MAX_FLATBUFFER_SIZE = 4096;

    /*!
     * @brief Constructs an XsensFullBodyTracker.
     * @param collection_id Logical stream identifier; must match the Xsens pusher's collection_id.
     * @param max_flatbuffer_size Upper bound for serialized ``FullBodyPosePico`` payloads.
     */
    explicit XsensFullBodyTracker(const std::string& collection_id,
                                  size_t max_flatbuffer_size = DEFAULT_MAX_FLATBUFFER_SIZE);

    std::string_view get_name() const override
    {
        return TRACKER_NAME;
    }

    /*!
     * @brief Full-body pose snapshot from the session's implementation.
     *
     * ``tracked.data`` is null when no valid sample exists / the collection is unavailable. When
     * non-null, the nested ``FullBodyPosePicoT`` (24 joints + all_joint_poses_tracked) is safe to
     * read.
     */
    const FullBodyPosePicoTrackedT& get_body_pose(const ITrackerSession& session) const;

    const std::string& collection_id() const
    {
        return collection_id_;
    }

    size_t max_flatbuffer_size() const
    {
        return max_flatbuffer_size_;
    }

private:
    static constexpr const char* TRACKER_NAME = "XsensFullBodyTracker";

    std::string collection_id_;
    size_t max_flatbuffer_size_;
};

} // namespace core
