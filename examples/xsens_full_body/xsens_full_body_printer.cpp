// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*!
 * @file xsens_full_body_printer.cpp
 * @brief Reader-side proof for the Xsens full-body add_device route (#3863, T4).
 *
 * Mirrors examples/schemaio/pedal_printer.cpp, but reads the Xsens full-body tensor collection via
 * the production reader facade core::XsensFullBodyTracker (-> LiveXsensFullBodyTrackerImpl ->
 * SchemaTracker), i.e. the exact path that feeds FullBodyInput in the retargeting engine — no Isaac
 * core change. It decodes each FullBodyPosePico frame the receiver-fed pusher
 * (src/plugins/xsens_full_body) pushes, and reports:
 *   - pelvis (joint 0) position (Pico Y-up; Y is height, ~0.96 m standing for testfile.mvn),
 *   - hand validity (joints 22/23 must be is_valid=false — MVN has no hand pose),
 *   - all_joint_poses_tracked (false — the production fingerprint),
 *   - distinct pelvis-Y count + min/max, so a frozen/cached stream is self-evident (liveness).
 *
 * Usage: xsens_full_body_printer [collection_id] [max_distinct_samples]
 *   collection_id        default "xsens_full_body" (must match the pusher)
 *   max_distinct_samples 0 (default) = run until SIGINT/SIGTERM (soak); N = stop after N distinct.
 */

#include <deviceio_session/deviceio_session.hpp>
#include <deviceio_trackers/xsens_full_body_tracker.hpp>
#include <oxr/oxr_session.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
std::atomic<bool> g_stop{ false };

extern "C" void on_signal(int /*sig*/)
{
    g_stop.store(true, std::memory_order_relaxed);
}

// Quantize pelvis Y to 0.1 mm to count distinct poses without float-equality noise.
int64_t quantize(float v)
{
    return static_cast<int64_t>(std::llround(static_cast<double>(v) * 10000.0));
}
} // namespace

int main(int argc, char** argv)
try
{
    const std::string collection_id = (argc > 1) ? argv[1] : "xsens_full_body";
    const long max_distinct = (argc > 2) ? std::atol(argv[2]) : 0; // 0 = run until interrupted

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "Xsens Full Body Printer (collection: " << collection_id
              << ", tensor: full_body_pose, max_distinct: " << max_distinct << ")" << std::endl;

    auto tracker = std::make_shared<core::XsensFullBodyTracker>(collection_id);

    std::vector<std::shared_ptr<core::ITracker>> trackers = { tracker };
    auto required_extensions = core::DeviceIOSession::get_required_extensions(trackers);
    auto oxr_session = std::make_shared<core::OpenXRSession>("XsensFullBodyPrinter", required_extensions);

    std::unique_ptr<core::DeviceIOSession> session = core::DeviceIOSession::run(trackers, oxr_session->get_handles());

    std::cout << "Reading... (Ctrl-C to stop)" << std::endl;

    uint64_t ticksWithData = 0;
    uint64_t distinct = 0;
    int64_t lastQ = std::numeric_limits<int64_t>::min();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    bool sawData = false;

    while (!g_stop.load(std::memory_order_relaxed))
    {
        session->update();

        const auto& tracked = tracker->get_body_pose(*session);
        if (!tracked.data || !tracked.data->joints)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // no sample yet: match push rate
            continue;
        }

        const auto* joints = tracked.data->joints->joints(); // const Array<BodyJointPose, 24>*
        const auto* pelvis = (*joints)[0];
        const float px = pelvis->pose().position().x();
        const float py = pelvis->pose().position().y();
        const float pz = pelvis->pose().position().z();
        const bool leftHandValid = (*joints)[22]->is_valid();
        const bool rightHandValid = (*joints)[23]->is_valid();
        const bool allTracked = tracked.data->all_joint_poses_tracked;

        ++ticksWithData;
        minY = std::min(minY, py);
        maxY = std::max(maxY, py);

        const int64_t q = quantize(py);
        if (q != lastQ) // a genuinely new pose (the backend retains the last sample between pushes)
        {
            lastQ = q;
            ++distinct;
            sawData = true;

            if (distinct == 1 || distinct % 60 == 0) // ~1 s at 60 fps
            {
                std::cout << std::fixed << std::setprecision(4) << "distinct=" << distinct << " pelvis=(" << px << ", "
                          << py << ", " << pz << ")"
                          << " handsValid(L,R)=(" << leftHandValid << "," << rightHandValid << ")"
                          << " allTracked=" << allTracked << std::endl;
            }

            if (max_distinct > 0 && distinct >= static_cast<uint64_t>(max_distinct))
                break;
        }
    }

    std::cout << "\n--- summary ---" << std::endl;
    std::cout << "ticks_with_data=" << ticksWithData << " distinct_pelvisY=" << distinct << std::endl;
    if (sawData)
    {
        std::cout << std::fixed << std::setprecision(4) << "pelvisY_min=" << minY << " pelvisY_max=" << maxY
                  << " (Pico Y-up; ~0.96 m standing for testfile.mvn; range>0 => tracking, not frozen)" << std::endl;
    }
    else
    {
        std::cout << "NO DATA received — collection not reachable or no frames pushed." << std::endl;
    }
    return sawData ? 0 : 2;
}
catch (const std::exception& e)
{
    std::cerr << argv[0] << ": " << e.what() << std::endl;
    return 1;
}
catch (...)
{
    std::cerr << argv[0] << ": Unknown error occurred" << std::endl;
    return 1;
}
