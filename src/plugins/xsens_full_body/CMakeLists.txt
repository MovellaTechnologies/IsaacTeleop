# SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Live receiver-fed pusher for the Isaac add_device Xsens full-body route (ADO #3862 / #3863).
#
# T1 (#3862) built the dummy-frame spike; #3863 embeds the UDP teleop receiver here (in-tree, one
# process — T2/OQ-2) so real MVN Studio frames flow receiver -> push_buffer. This plugin therefore
# compiles three MVN picofullbody_core TUs directly: the receiver's framing (teleop_wire.cpp) and
# verify gate (picofullbody_converter.cpp), plus the embedded receiver (teleop_receiver.cpp). The
# MVN paths auto-detect from the standard trunk layout (<trunk>/{IsaacTeleop,mvn,3p,linux-x64}) and
# can be overridden via cache vars. If the MVN converter is not found the plugin is skipped with a
# message rather than failing the whole IsaacTeleop build (it is a cross-repo dev/reference target,
# not a core deliverable).

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "Skipping xsens_full_body plugin (Linux only)")
    add_custom_target(xsens_full_body_plugin
        COMMAND ${CMAKE_COMMAND} -E echo "Skipping xsens_full_body: Linux only")
    return()
endif()

set(XSENS_MVN_PICOFULLBODY_DIR "${CMAKE_SOURCE_DIR}/../mvn/mvn_studio/src/picofullbody_core"
    CACHE PATH "Path to the MVN picofullbody_core dir (converter + schema, source-only compile input)")
set(XSENS_3P_FLATBUFFERS_INCLUDE "${CMAKE_SOURCE_DIR}/../3p/flatbuffers/include"
    CACHE PATH "Path to the MVN 3p FlatBuffers include dir (must be the pinned 24.3.25)")
set(XSENS_XLIB_INCLUDE "${CMAKE_SOURCE_DIR}/../linux-x64/include"
    CACHE PATH "Path to the MVN linux-x64 include dir (provides common/dbc.h)")

set(_pfb "${XSENS_MVN_PICOFULLBODY_DIR}")
if(NOT EXISTS "${_pfb}/picofullbody_converter.cpp")
    message(STATUS "Skipping xsens_full_body plugin: MVN converter not found at ${_pfb} "
                   "(set -DXSENS_MVN_PICOFULLBODY_DIR=... to enable)")
    add_custom_target(xsens_full_body_plugin
        COMMAND ${CMAKE_COMMAND} -E echo "Skipping xsens_full_body: MVN converter not found")
    return()
endif()

add_executable(xsens_full_body_plugin
    main.cpp
    xsens_full_body_plugin.cpp
    teleop_receiver.cpp
    "${_pfb}/picofullbody_converter.cpp"
    "${_pfb}/teleop_wire.cpp"
)

# The plugin TUs find their own private headers (teleop_receiver.h, xsens_full_body_plugin.hpp) via
# CMAKE_CURRENT_SOURCE_DIR; the receiver + converter TUs find picofullbody_converter.h / teleop_wire.h
# in ${_pfb}, the generated schema headers in ${_pfb}/schema, and transitively common/dbc.h + flatbuffers.
target_include_directories(xsens_full_body_plugin PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${_pfb}"
    "${_pfb}/schema"
    "${XSENS_3P_FLATBUFFERS_INCLUDE}"
    "${XSENS_XLIB_INCLUDE}"
)

# Deliberately NOT linking isaacteleop_schema: the pusher forwards MVN-produced bytes verbatim and
# never touches Isaac's core::FullBodyPosePico types. Compiling the MVN full_body_generated.h (also
# namespace core) alongside isaacteleop_schema would be an ODR clash. pusherio + oxr_core is all the
# push surface needs; Threads for the receiver's std::atomic/socket use.
find_package(Threads REQUIRED)
target_link_libraries(xsens_full_body_plugin PRIVATE
    pusherio::pusherio
    oxr::oxr_core
    Threads::Threads
)

install(TARGETS xsens_full_body_plugin RUNTIME DESTINATION plugins/xsens_full_body)
install(FILES plugin.yaml README.md DESTINATION plugins/xsens_full_body)
