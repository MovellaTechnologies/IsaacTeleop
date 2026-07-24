#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 Xsens Technologies B.V. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Standalone build of the Xsens full-body receiver-fed pusher (#3863), mirroring
# mvn_isaac_devtools/tools/teleop_udp/build.sh. Compiles the embedded UDP receiver
# (teleop_receiver.cpp) + the MVN framing/verify TUs (teleop_wire.cpp, picofullbody_converter.cpp)
# against the IsaacTeleop build-tree static libs + headers, so it can be built and run without
# reconfiguring the whole IsaacTeleop super-build.
#
# The in-tree CMake target (add_subdirectory(src/plugins/xsens_full_body)) is the eventual home;
# this script is the fast T1 iteration path. Both produce the same executable.
#
# Paths derive from this script's location (standard trunk layout: <trunk>/{IsaacTeleop,mvn,3p,
# linux-x64}). Override IT_ROOT / MVN_ROOT / THREEP / XLIB / IT_BUILD via env.
set -eu
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IT_ROOT="${IT_ROOT:-$(cd "$HERE/../../.." && pwd)}"
IT_BUILD="${IT_BUILD:-$IT_ROOT/build}"
MVN_ROOT="${MVN_ROOT:-$(cd "$IT_ROOT/../mvn" && pwd)}"
THREEP="${THREEP:-$(cd "$MVN_ROOT/../3p" && pwd)}"
XLIB="${XLIB:-$(cd "$MVN_ROOT/../linux-x64" && pwd)}"
PFB="$MVN_ROOT/mvn_studio/src/picofullbody_core"

echo "IT_ROOT=$IT_ROOT"
echo "IT_BUILD=$IT_BUILD"
echo "MVN_ROOT=$MVN_ROOT"

# IsaacTeleop headers (source tree) + OpenXR headers (build-tree fetch) + NVX1 extension headers.
IT_INC=(
    -I"$IT_ROOT/src/core/pusherio/cpp/inc"
    -I"$IT_ROOT/src/core/oxr/cpp/inc"
    -I"$IT_ROOT/src/core/oxr_utils/cpp/inc"
    -I"$IT_BUILD/_deps/openxr-sdk-src/include"
    -I"$IT_ROOT/deps/cloudxr/openxr_extensions"
)

# IsaacTeleop build-tree static libs (built once via `cmake --build build`). Order: our objects use
# pusherio + oxr_core; both use the OpenXR loader; loader needs dl/pthread/rt.
IT_LIBS=(
    "$IT_BUILD/src/core/pusherio/cpp/libpusherio.a"
    "$IT_BUILD/src/core/oxr/cpp/liboxr_core.a"
    "$IT_BUILD/_deps/openxr-sdk-build/src/loader/libopenxr_loader.a"
)

for lib in "${IT_LIBS[@]}"; do
    if [ ! -f "$lib" ]; then
        echo "ERROR: missing $lib — build IsaacTeleop first: cmake --build \"$IT_BUILD\"" >&2
        exit 1
    fi
done

# -DXR_USE_TIMESPEC selects the Linux timespec path in oxr_utils/oxr_time.hpp (the in-tree CMake
# build gets this transitively from the oxr targets; the standalone build must pass it explicitly).
g++ -std=c++20 -O2 -DXR_USE_TIMESPEC -o "$HERE/xsens_full_body_plugin" \
    "$HERE/main.cpp" "$HERE/xsens_full_body_plugin.cpp" "$HERE/teleop_receiver.cpp" \
    "$PFB/picofullbody_converter.cpp" "$PFB/teleop_wire.cpp" \
    -I"$HERE" -I"$PFB" -I"$PFB/schema" -I"$THREEP/flatbuffers/include" -I"$XLIB/include" \
    "${IT_INC[@]}" \
    "${IT_LIBS[@]}" \
    -ldl -lpthread -lrt
echo "built $HERE/xsens_full_body_plugin"
