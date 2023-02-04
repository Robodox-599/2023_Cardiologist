
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "subsystems/subsystem_DriveTrain.h"
#include "frc/smartdashboard/SmartDashboard.h"



subsystem_DriveTrain::subsystem_DriveTrain():
    m_Gyro{SwerveConstants::CANCoderID}, 
    m_FrontLeftModule{FrontLeftModule::Constants},
    m_FrontRightModule{FrontRightModule::Constants},
    m_BackLeftModule{BackLeftModule::Constants},
    m_BackRightModule{BackRightModule::Constants},
    m_PoseEstimator{m_kinematics,
      frc::Rotation2d{},
      {m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(),
       m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition()},
      frc::Pose2d{}}, 
    m_PID{SwerveConstants::BalancekP, SwerveConstants::BalancekI, SwerveConstants::BalancekD}
{
    frc::Wait(1_s);
    ResetModulesToAbsolute();
    m_Gyro.ConfigFactoryDefault();
    ZeroGyro();
    DegreeOfThrottle = 1;


}

void subsystem_DriveTrain::SwerveDrive(units::meters_per_second_t xSpeed,
                                        units::meters_per_second_t ySpeed,
                                        units::radians_per_second_t zRot,
                                        bool FieldRelative, 
                                        bool IsOpenLoop){

    if(StartBalance){
        // xSpeed += CalculateRoll();
        // ySpeed += CalculatePitch();
    }
    auto moduleStates = SwerveConstants::m_kinematics.ToSwerveModuleStates(
        FieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, zRot, GetYaw()
                        ): frc::ChassisSpeeds{xSpeed, ySpeed, zRot});
    SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&moduleStates, SwerveConstants::MaxSpeed);
    auto [FrontLeft, FrontRight, BackLeft, BackRight] = moduleStates;
    //auto [FrontRight, RearRight,]

     m_FrontLeftModule.SetDesiredState(FrontLeft, IsOpenLoop);
    m_FrontRightModule.SetDesiredState(FrontRight, IsOpenLoop);
     m_BackLeftModule.SetDesiredState(BackLeft, IsOpenLoop);
    m_BackRightModule.SetDesiredState(BackRight, IsOpenLoop);

}



void subsystem_DriveTrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates){
  SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::MaxSpeed);
  m_FrontLeftModule.SetDesiredState(desiredStates[0], false);
  m_FrontRightModule.SetDesiredState(desiredStates[1], false);
  m_BackLeftModule.SetDesiredState(desiredStates[2], false);
  m_BackRightModule.SetDesiredState(desiredStates[3], false);

}



double subsystem_DriveTrain::SetThrottle(double input){
    // double shiftedThrottle = input < 0.0 ? DegreeOfThrottle == 1 ? input : -1 * pow(input, DegreeOfThrottle) : pow(input, DegreeOfThrottle);

    // if(DegreeOfThrottle == 1){
    //     return input;
    // } else {
    //     return input < 0.0 ? -1 * pow(input, DegreeOfThrottle) : pow(input, DegreeOfThrottle);
    // }
    frc::SmartDashboard::SmartDashboard::PutNumber("DegreeOfThrottle",DegreeOfThrottle);
    if(DegreeOfThrottle != 2){
        return input;
    } else {
        if(input < 0.0){
            return -1 * pow(input, DegreeOfThrottle);
        } else {
            return pow(input, DegreeOfThrottle);
        }
    }
    
    }

void subsystem_DriveTrain::ChangeThrottle(){
    if( DegreeOfThrottle == SwerveConstants::LinearThrottle){
        DegreeOfThrottle = SwerveConstants::NonLinearThrottle;
    }else{
        DegreeOfThrottle = SwerveConstants::LinearThrottle;
    }
}

void subsystem_DriveTrain::ResetOdometry(frc::Rotation2d Rotation, frc::Pose2d Pose){
    ZeroGyro();
    m_PoseEstimator.ResetPosition(Rotation,
                            {m_FrontLeftModule.GetPosition(), 
                                m_FrontRightModule.GetPosition(),
                                m_BackLeftModule.GetPosition(),
                                m_BackRightModule.GetPosition()},
                            Pose);
}


frc::Pose2d subsystem_DriveTrain::GetPose(){
    return m_PoseEstimator.GetEstimatedPosition();
}

frc::Rotation2d subsystem_DriveTrain::GetYaw(){
    
    units::degree_t Yaw{ m_Gyro.GetYaw() };
    return (SwerveConstants::InvertGyro) ? frc::Rotation2d{360_deg - Yaw}: frc::Rotation2d{Yaw}; 
}

units::meters_per_second_t subsystem_DriveTrain::CalculatePitch(){
    
    return units::meters_per_second_t{m_PID.Calculate(m_Gyro.GetPitch())};
}

units::meters_per_second_t subsystem_DriveTrain::CalculateRoll(){
    return units::meters_per_second_t{m_PID.Calculate(m_Gyro.GetRoll())};
}

void subsystem_DriveTrain::SetStationBalance(){
    StartBalance = !StartBalance;
}

void subsystem_DriveTrain::ZeroGyro(){
    
    m_Gyro.SetYaw(0);

}

void subsystem_DriveTrain::ImplementVisionPose(std::pair<frc::Pose2d, units::millisecond_t> pair){
        m_PoseEstimator.AddVisionMeasurement(pair.first, pair.second);
}

void subsystem_DriveTrain::ResetModulesToAbsolute(){
    m_FrontLeftModule.ResetToAbsolute();
    m_FrontRightModule.ResetToAbsolute();
    m_BackLeftModule.ResetToAbsolute();
    m_BackRightModule.ResetToAbsolute();
}

// void subsystem_DriveTrain::



// This method will be called once per scheduler run
void subsystem_DriveTrain::Periodic() {

    frc::SmartDashboard::SmartDashboard::PutNumber("X Position", m_PoseEstimator.GetEstimatedPosition().X().value());
    frc::SmartDashboard::SmartDashboard::PutNumber("Y Position", m_PoseEstimator.GetEstimatedPosition().Y().value());
    frc::SmartDashboard::SmartDashboard::PutNumber("Rotation", m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value()); 

    double velFL = m_FrontLeftModule.GetState().speed.value();
    double velFR = m_FrontRightModule.GetState().speed.value();
    double velBL = m_BackLeftModule.GetState().speed.value();
    double velBR = m_BackRightModule.GetState().speed.value();
    double average = velFL + velFR + velBL + velBR;
    
    // frc::SmartDashboard::SmartDashboard::PutNumber("IsSimilarFL", fabs(average - velFL)/average);
    // frc::SmartDashboard::SmartDashboard::PutNumber("IsSimilarFR", fabs(average - velFR)/average);
    // frc::SmartDashboard::SmartDashboard::PutNumber("IsSimilarBL", fabs(average - velBL)/average);
    // frc::SmartDashboard::SmartDashboard::PutNumber("IsSimilarBR", fabs(average - velBR)/average);   


    // frc::SmartDashboard::SmartDashboard::PutNumber("FrontLeftSpeed", velFL);   
    // frc::SmartDashboard::SmartDashboard::PutNumber("FrontRightSpeed", velFR);   
    // frc::SmartDashboard::SmartDashboard::PutNumber("BackLeftSpeed", velBL);   
    // frc::SmartDashboard::SmartDashboard::PutNumber("BackRightSpeed", velBR);

    // frc::SmartDashboard::SmartDashboard::PutNumber("FL - FR", velFL - velFR);
    // frc::SmartDashboard::SmartDashboard::PutNumber("FL - BL", velFL - velBL);
    // frc::SmartDashboard::SmartDashboard::PutNumber("FL - BR", velFL - velBR);
    // frc::SmartDashboard::SmartDashboard::PutNumber("FR - BL", velFR - velBL);
    // frc::SmartDashboard::SmartDashboard::PutNumber("FR - BR", velFR - velBR);
    // frc::SmartDashboard::SmartDashboard::PutNumber("BL - BR", velBL - velBR);   

    m_PoseEstimator.Update(m_Gyro.GetRotation2d(),
                      {m_FrontLeftModule.GetPosition(), 
                                m_FrontRightModule.GetPosition(),
                                m_BackLeftModule.GetPosition(),
                                m_BackRightModule.GetPosition()} );

}

