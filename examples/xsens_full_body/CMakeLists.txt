# SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)

find_package(Threads REQUIRED)

# Reader-side proof for the Xsens full-body add_device route (#3863, T4). Consumes the tensor
# collection via the production reader facade core::XsensFullBodyTracker (deviceio_trackers) driven
# by a DeviceIOSession (which pulls live_trackers transitively). Mirrors examples/schemaio.
add_executable(xsens_full_body_printer
    xsens_full_body_printer.cpp
)

target_link_libraries(xsens_full_body_printer PRIVATE
    deviceio::deviceio_session
    deviceio::deviceio_trackers
    oxr::oxr_core
    isaacteleop_schema
    ${CMAKE_DL_LIBS}
    Threads::Threads
)

install(TARGETS xsens_full_body_printer
    RUNTIME DESTINATION examples/xsens_full_body
)
