// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "schema_tracker_base.hpp"

#include <flatbuffers/flatbuffers.h>
#include <mcap/tracker_channels.hpp>

#include <memory>
#include <optional>

namespace core
{

/**
 * @brief Typed SchemaTracker that optionally records to MCAP.
 *
 * Wraps SchemaTrackerBase with FlatBuffer type knowledge so that each sample
 * read from the tensor can be automatically written to an MCAP channel.
 *
 * @tparam RecordT    FlatBuffer record wrapper (e.g. Generic3AxisPedalOutputRecord).
 * @tparam DataTableT FlatBuffer data table (e.g. Generic3AxisPedalOutput).
 */
template <typename RecordT, typename DataTableT>
class SchemaTracker : public SchemaTrackerBase
{
public:
    using NativeDataT = typename DataTableT::NativeTableType;
    using Channels = McapTrackerChannels<RecordT, DataTableT>;

    /**
     * @param mcap_channels Non-owning pointer to the MCAP channel writer. Must outlive
     *        this SchemaTracker. Owned by the live tracker impl that also owns this
     *        SchemaTracker instance. Null when recording is disabled.
     * @param mcap_channel_index 0-based sub-channel index within mcap_channels
     *        used for per-sample recording.
     * @param mcap_channel_tracked_index If set, an additional write of only the final
     *        sample per update() call is made to this sub-channel index within the
     *        same mcap_channels. Unset to disable.
     */
    SchemaTracker(const OpenXRSessionHandles& handles,
                  SchemaTrackerConfig config,
                  Channels* mcap_channels = nullptr,
                  size_t mcap_channel_index = 0,
                  std::optional<size_t> mcap_channel_tracked_index = std::nullopt)
        : SchemaTrackerBase(handles, std::move(config)),
          mcap_channels_(mcap_channels),
          mcap_channel_index_(mcap_channel_index),
          mcap_channel_tracked_index_(mcap_channel_tracked_index)
    {
    }

    /**
     * @brief Read all pending samples; write each to MCAP if channels are set.
     *
     * Each sample is unpacked, repacked into a Record with its timestamp,
     * and written to the MCAP channel. The last sample's unpacked data is
     * returned via out_latest (if non-null and samples were read).
     *
     * @param out_latest If non-null and samples were read, receives the unpacked
     *                   data from the last sample. Cleared when the tensor collection
     *                   is absent.
     * @throws std::runtime_error On critical OpenXR/tensor API failures propagated
     *         from SchemaTrackerBase.
     * @note Missing collection, temporary collection loss, and "no new sample"
     *       are treated as common non-fatal conditions and do not throw.
     */
    void update(std::shared_ptr<NativeDataT>& out_latest)
    {
        samples_.clear();
        bool present = read_all_samples(samples_);
        last_sample_count_ = samples_.size();

        if (samples_.empty())
        {
            if (!present)
            {
                out_latest.reset();
            }
            return;
        }

        DeviceDataTimestamp last_timestamp{};
        for (const auto& sample : samples_)
        {
            auto fb = flatbuffers::GetRoot<DataTableT>(sample.buffer.data());
            if (!fb)
            {
                continue;
            }

            if (!out_latest)
            {
                out_latest = std::make_shared<NativeDataT>();
            }
            fb->UnPackTo(out_latest.get());
            last_timestamp = sample.timestamp;

            // write() serializes synchronously and does not retain the shared_ptr,
            // so reusing out_latest across loop iterations is safe.
            if (mcap_channels_)
            {
                mcap_channels_->write(mcap_channel_index_, sample.timestamp, out_latest);
            }
        }

        if (mcap_channel_tracked_index_ && mcap_channels_ && out_latest)
        {
            mcap_channels_->write(*mcap_channel_tracked_index_, last_timestamp, out_latest);
        }

        last_timestamp_ = last_timestamp;
    }

    /**
     * @brief Timestamp of the last sample drained by update(), zero-initialised until the
     *        first sample arrives and retained across ticks that drain nothing.
     *
     * Both clocks are already local monotonic nanoseconds (see SchemaTrackerBase):
     * sample_time_local_common_clock is the value the producer passed to
     * SchemaPusher::push_buffer, available_time_local_common_clock is when the sample became
     * available here. Callers detect "a new sample arrived this tick" via last_sample_count(),
     * or equivalently by sample_time_local_common_clock changing.
     */
    const DeviceDataTimestamp& last_timestamp() const
    {
        return last_timestamp_;
    }

    //! Number of samples drained by the most recent update() call (0 when it drained none).
    //! Greater than 1 means the consumer is polling slower than the producer pushes.
    size_t last_sample_count() const
    {
        return last_sample_count_;
    }

private:
    Channels* mcap_channels_;
    size_t mcap_channel_index_;
    std::optional<size_t> mcap_channel_tracked_index_;
    std::vector<SampleResult> samples_;
    DeviceDataTimestamp last_timestamp_{};
    size_t last_sample_count_ = 0;
};

} // namespace core
