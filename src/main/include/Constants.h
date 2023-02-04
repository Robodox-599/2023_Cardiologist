// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/util/Color.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace IntakeConstants {
    constexpr int IntakeMotorID = 4;
    constexpr double OutputPower = -0.4;
    constexpr units::second_t TimerConstant {0.2};
    
    constexpr int IntakePistonA = 5;
    constexpr int IntakePistonB = 4;
}

namespace ControllerConstants {
    constexpr int XboxPort = 0;
    constexpr int XboxLTAxis = 2;
    constexpr int XboxRTAxis = 3;
}

namespace ColorConstants {
    constexpr frc::Color PurpleTarget = frc::Color(0.180, 0.339, 0.479);
    constexpr frc::Color YellowTarget = frc::Color(0.361, 0.524, 0.113);

    constexpr uint32_t ProximityTarget = 100;
}