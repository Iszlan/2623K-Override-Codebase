#pragma once

namespace klib {

    struct MoveToPointParams {
        bool forwards = true;
        double maxVoltage = 12.0;
        double minVoltage = 0.0;
        double earlyExitRange = 0.0;
        double settleDistance = 1.0;
        int settleTime = 100;
    };

} // klib
