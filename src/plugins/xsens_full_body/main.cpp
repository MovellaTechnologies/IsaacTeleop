// SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "xsens_full_body_plugin.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace plugins::xsens_full_body;

namespace
{
std::atomic<bool> g_stop{ false };

extern "C" void on_signal(int /*sig*/)
{
    g_stop.store(true, std::memory_order_relaxed);
}
} // namespace

int main(int argc, char** argv)
try
{
    const std::string collection_id = (argc > 1) ? argv[1] : "xsens_full_body";
    const uint16_t port = (argc > 2) ? static_cast<uint16_t>(std::atoi(argv[2])) : XsensFullBodyPlugin::DEFAULT_PORT;

    // Clean shutdown for the soak: SIGINT/SIGTERM set the stop flag so run() returns, the receiver
    // closes its socket, and final stats print. Killing MVN Studio does NOT stop the pusher — the
    // receiver treats the next seq=0 as a session boundary (the session-restart AC).
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "Xsens Full Body Pusher (collection: " << collection_id
              << ", tensor: full_body_pose, udp: 0.0.0.0:" << port << ")" << std::endl;

    XsensFullBodyPlugin plugin(collection_id, port);
    plugin.run(g_stop); // blocks until a signal or a fatal socket error

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
