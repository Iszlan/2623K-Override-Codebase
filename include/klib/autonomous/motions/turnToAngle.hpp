#pragma once

namespace klib {

    struct TurnToAngleParams {
        double maxVoltage = 12;
        double minVoltage = 0;
        double earlyExitRange = 0;
        double settleError = 0.5;
        int settleTime = 100;
    };

} // klib
