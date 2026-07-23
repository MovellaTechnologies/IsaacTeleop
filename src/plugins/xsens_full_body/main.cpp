// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "xsens_full_body_plugin.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <string>
#include <thread>

using namespace plugins::xsens_full_body;

int main(int argc, char** argv)
try
{
    if (argc == 0)
    {
        std::cerr << "Usage: " << argv[0] << " [collection_id]" << std::endl;
        return 1;
    }

    const std::string collection_id = (argc > 1) ? argv[1] : "xsens_full_body";

    std::cout << "Xsens Full Body Pusher (collection: " << collection_id << ", tensor: full_body_pose)"
              << std::endl;

    XsensFullBodyPlugin plugin(collection_id);

    // Push dummy animated frames at ~250 Hz (T1 S5).
    const auto frame_duration = std::chrono::nanoseconds(1000000000 / 250);
    const auto program_start = std::chrono::steady_clock::now();
    std::size_t frame_count = 0;

    while (true)
    {
        plugin.update();
        frame_count++;
        std::this_thread::sleep_until(program_start + frame_duration * frame_count);
    }

    return 0;
}
catch (const std::exception& e)
{
    std::cerr << argv[0] << ": " << e.what() << std::endl;
    return 1;
}
catch (...)
{
    std::cerr << argv[0] << ": Unknown error" << std::endl;
    return 1;
}
