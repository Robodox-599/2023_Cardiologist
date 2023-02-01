#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>
#include "Constants.h"
class HardwareConfig{
    public:
    ctre::phoenix::sensors::CANCoderConfiguration SwerveCanCoderConfig;
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration SwerveAngleFXConfig;
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration SwerveDriveFXConfig;

    HardwareConfig(){
        /*Swerve CANCoder Config*/
        SwerveCanCoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
        SwerveCanCoderConfig.sensorDirection = SwerveConstants::CanCoderInvert;
        SwerveCanCoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
        SwerveCanCoderConfig.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond;

        /*Swerve Drive Motor Config*/
        
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration DriveSupplyLimit{SwerveConstants::DriveEnableCurrentLimit,
                                                                                      SwerveConstants::DriveContinuousCurrentLimit,
                                                                                      SwerveConstants::DrivePeakCurrentLimit,
                                                                                      SwerveConstants::DrivePeakCurrentDuration};
        
        SwerveDriveFXConfig.slot0.kP = SwerveConstants::DriveKP;
        SwerveDriveFXConfig.slot0.kI = SwerveConstants::DriveKI;
        SwerveDriveFXConfig.slot0.kD = SwerveConstants::DriveKD;
        SwerveDriveFXConfig.supplyCurrLimit = DriveSupplyLimit;
        SwerveDriveFXConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
        SwerveDriveFXConfig.openloopRamp = SwerveConstants::OpenLoopRamp;
        SwerveDriveFXConfig.closedloopRamp = SwerveConstants::closedLoopRamp;
        SwerveDriveFXConfig.voltageCompSaturation = SwerveConstants::kNominal.value();

        /*Swerve Angle Motor Config*/
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration AngleSupplyLimit{SwerveConstants::AngleEnableCurrentLimit,
                                                                                      SwerveConstants::AngleContinuousCurrentLimit,
                                                                                      SwerveConstants::AnglePeakCurrentLimit,
                                                                                      SwerveConstants::AnglePeakCurrentDuration};
        SwerveAngleFXConfig.slot0.kP = SwerveConstants::AngleKP;
        SwerveAngleFXConfig.slot0.kI = SwerveConstants::AngleKI;
        SwerveAngleFXConfig.slot0.kD = SwerveConstants::AngleKD;
        SwerveAngleFXConfig.supplyCurrLimit = AngleSupplyLimit;
        SwerveAngleFXConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;

        
    }
    


};