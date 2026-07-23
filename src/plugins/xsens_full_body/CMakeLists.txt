# SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# T1 spike pusher for the Isaac add_device Xsens full-body route (ADO #3862).
#
# This plugin compiles the MVN dummy-frame builder (picofullbody_converter.cpp) directly, so it
# needs three paths from the sibling MVN checkout / third-party tree. They auto-detect from the
# standard trunk layout (<trunk>/{IsaacTeleop,mvn,3p,linux-x64}) and can be overridden via cache
# vars. If the MVN converter is not found the plugin is skipped with a message rather than failing
# the whole IsaacTeleop build (it is a cross-repo dev/reference target, not a core deliverable).

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
    "${_pfb}/picofullbody_converter.cpp"
)

# The plugin TUs (main/plugin) find picofullbody_converter.h here; the converter TU finds its
# generated schema headers and, transitively, common/dbc.h + flatbuffers.
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
# push surface needs.
target_link_libraries(xsens_full_body_plugin PRIVATE
    pusherio::pusherio
    oxr::oxr_core
)

install(TARGETS xsens_full_body_plugin RUNTIME DESTINATION plugins/xsens_full_body)
install(FILES plugin.yaml README.md DESTINATION plugins/xsens_full_body)
