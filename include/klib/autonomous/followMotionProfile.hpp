#pragma once
#include "klib/autonomous/motion/ramsete.hpp"

namespace klib {

    enum class FeedbackMode {
        NONE,
        HEADING_PID,
        RAMSETE
    };

    struct MotionProfileFollowerParams {
        double maxVoltage = 12.0;

        double kS = 0.0;
        double kV = 0.0;
        double kA = 0.0;

        double trackWidth = 10.75;

        FeedbackMode feedbackMode = FeedbackMode::RAMSETE;
        klib::motion::RamseteGains ramseteGains{};
    };

} // klib
