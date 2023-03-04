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
    // Intake Motor
    constexpr int IntakeMotorID = 4;
    constexpr double OuttakePower = 0.3;
    constexpr double IntakePower = -0.6;
    constexpr double PassivePower = -0.05;


    // PID stuff
    constexpr double kIntakeP = 0.1;
    constexpr double kIntakeD = 1.0;
    constexpr double kIntakeFF = 0.00025 / 1.6;
    // Constant for amount of time in transitioning between wheels on and toggle clamp
    constexpr units::second_t TimerConstant {0.2};
    
    constexpr int IntakePistonA = 5;
    constexpr int IntakePistonB = 4;

    constexpr double OutputCurrent = 20;

    // Conversion rate for velocity based on proximity
    constexpr double ProxToVelocity = 50.0;

    constexpr int CurrentLimit = 10;
    constexpr double MaxVelocity = 30.0;
    constexpr double kProximityP = 0.005;
    constexpr double kProximityD = 0.0001;

    enum IntakeMode{
        Intake = 0,
        Outake = 1,
        Passive = 2,
        Off = 3
    };
}

namespace ControllerConstants {
    constexpr int XboxPort = 0;
}

namespace ColorConstants {
    constexpr frc::Color PurpleTarget = frc::Color(0.202, 0.333, 0.459);
    constexpr frc::Color YellowTarget = frc::Color(0.361, 0.524, 0.113);

    constexpr double RecognitionProximity = 80.0;
    constexpr double TargetProximity = 20.0;
}