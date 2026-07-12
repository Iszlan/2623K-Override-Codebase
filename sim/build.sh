#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

CORE_INCLUDE="../include/klib/autonomous/motion"
CORE_SRC="../src/klib/autonomous/motion"

g++ -std=c++17 -O2 -Wall -Wextra \
    -I"$CORE_INCLUDE" \
    "$CORE_SRC"/cubicBezier.cpp \
    "$CORE_SRC"/path.cpp \
    "$CORE_SRC"/trajectory.cpp \
    "$CORE_SRC"/motionProfile.cpp \
    "$CORE_SRC"/ramsete.cpp \
    simulate.cpp \
    -o simulate

g++ -std=c++17 -O2 -Wall -Wextra \
    -I"$CORE_INCLUDE" \
    "$CORE_SRC"/cubicBezier.cpp \
    "$CORE_SRC"/path.cpp \
    "$CORE_SRC"/trajectory.cpp \
    "$CORE_SRC"/motionProfile.cpp \
    "$CORE_SRC"/ramsete.cpp \
    simulateRamsete.cpp \
    -o simulateRamsete

echo "Build OK -> ./simulate, ./simulateRamsete"
