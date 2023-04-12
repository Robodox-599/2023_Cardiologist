
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "subsystems/subsystem_DriveTrain.h"
#include "frc/smartdashboard/SmartDashboard.h"



subsystem_DriveTrain::subsystem_DriveTrain():
    m_Gyro{SwerveConstants::CANCoderID,"DriveCANivore"}, 
    m_FrontLeftModule{FrontLeftModule::Constants},
    m_FrontRightModule{FrontRightModule::Constants},
    m_BackLeftModule{BackLeftModule::Constants},
    m_BackRightModule{BackRightModule::Constants},
    m_PoseEstimator{m_kinematics,
      frc::Rotation2d{},
      {m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(),
       m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition()},
      frc::Pose2d{}, {0.70, 0.70, 0.10}, {0.30, 0.30, 0.90}}, 
    m_PitchCorrectionPID{0.000, 0.0, 0},
    m_RollCorrectionPID{0.01, 0.0, 0}
{
    frc::Wait(1_s);
    ResetModulesToAbsolute();
    m_Gyro.ConfigFactoryDefault();
    ZeroGyro();
    DegreeOfThrottle = SwerveConstants::Throttle::NONLINEAR;
    m_IsBalancing = false;
    m_IsAutoOrient = false;

    m_PitchCorrectionPID.SetSetpoint(0.0);
    m_PitchCorrectionPID.SetTolerance(2);


    m_RollCorrectionPID.SetSetpoint(0.0);
    m_RollCorrectionPID.SetTolerance(2);

    m_AutoOrientPID.EnableContinuousInput(-180, 180);
    m_AutoOrientPID.SetTolerance(1);

    // m_ProfiledOrientPID.EnableContinuousInput(-180_deg, 180_deg);
    // m_ProfiledOrientPID.SetTolerance(1_deg);


    // m_LEDTimer.Start();

    // cameras.push_back(std::make_pair(cameraOne, robotToCam));

    // SetAllianceOrigin(m_Alliance);
    // m_Chooser.SetDefaultOption("BLUE_ALLIANCE", BLUE_ALLIANCE);
    // m_Chooser.AddOption("RED_ALLIANCE", RED_ALLIANCE);
    // frc::SmartDashboard::PutNumber("ID DETECTED", 0);

    // frc::SmartDashboard::PutData(&m_Chooser);
}

void subsystem_DriveTrain::SwerveDrive(units::meters_per_second_t xSpeed,
                                        units::meters_per_second_t ySpeed,
                                        units::radians_per_second_t zRot,
                                        bool FieldRelative, 
                                        bool IsOpenLoop){

    // if( IsBalanced()){
    //     m_IsBalancing = false;
    // }
    if( m_IsBalancing  ){
        xSpeed += AddRollCorrection() * GetPoseYaw().Cos();
    }
    


    zRot = m_IsAutoOrient ? GetAngularVelocity() :  zRot;
    
    auto moduleStates = SwerveConstants::m_kinematics.ToSwerveModuleStates(
        FieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, zRot, GetPoseYaw()
                        ): frc::ChassisSpeeds{xSpeed, ySpeed, zRot});
    SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&moduleStates, SwerveConstants::MaxSpeed);
    auto [FrontLeft, FrontRight, BackLeft, BackRight] = moduleStates;
    //auto [FrontRight, RearRight,]

    m_FrontLeftModule.SetDesiredState(FrontLeft, IsOpenLoop);
    m_FrontRightModule.SetDesiredState(FrontRight, IsOpenLoop);
    m_BackLeftModule.SetDesiredState(BackLeft, IsOpenLoop);
    m_BackRightModule.SetDesiredState(BackRight, IsOpenLoop);

}

bool subsystem_DriveTrain::IsBalanced(){
    return (fabs(m_Gyro.GetPitch()) < 2) && (fabs(m_Gyro.GetRoll()) < 2);
}





void subsystem_DriveTrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates){
  SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::MaxSpeed);
  m_FrontLeftModule.SetDesiredState(desiredStates[0], false);
  m_FrontRightModule.SetDesiredState(desiredStates[1], false);
  m_BackLeftModule.SetDesiredState(desiredStates[2], false);
  m_BackRightModule.SetDesiredState(desiredStates[3], false);

}

void subsystem_DriveTrain::SetPark(){
    m_FrontLeftModule.SetDesiredAngle(frc::Rotation2d{-45_deg});
    m_FrontRightModule.SetDesiredAngle(frc::Rotation2d{45_deg});
    m_BackLeftModule.SetDesiredAngle(frc::Rotation2d{45_deg});
    m_BackRightModule.SetDesiredAngle(frc::Rotation2d{-45_deg});
}



double subsystem_DriveTrain::SetThrottle(double input){
    // double shiftedThrottle = input < 0.0 ? DegreeOfThrottle == 1 ? input : -1 * pow(input, DegreeOfThrottle) : pow(input, DegreeOfThrottle);

    // if(DegreeOfThrottle == 1){
    //     return input;
    // } else {
    //     return input < 0.0 ? -1 * pow(input, DegreeOfThrottle) : pow(input, DegreeOfThrottle);
    // }
    if(DegreeOfThrottle != SwerveConstants::Throttle::NONLINEAR){
        return input;
    } else {
        if(input < 0.0){
            // return -1 * pow(input, DegreeOfThrottle);
            return -1 * ( 1.01 * pow(input, 2) - 0.0202 * input + 0.0101);
        } else {
            // return pow(input, DegreeOfThrottle);
            return ( 1.01 * pow(input, 2) - 0.0202 * input + 0.0101);
        }
    }
    
    }

void subsystem_DriveTrain::ChangeThrottle(){
    if( DegreeOfThrottle == SwerveConstants::Throttle::LINEAR){
        DegreeOfThrottle = SwerveConstants::Throttle::NONLINEAR;
    }else{
        DegreeOfThrottle = SwerveConstants::Throttle::LINEAR;
    }
}

frc2::CommandPtr subsystem_DriveTrain::ToggleThrottleCommand(){
    return RunOnce( [this]{ ChangeThrottle();});
}

void subsystem_DriveTrain::ResetOdometry(frc::Pose2d Pose){
    m_PoseEstimator.ResetPosition(m_Gyro.GetRotation2d(),
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

frc::Rotation2d subsystem_DriveTrain::GetPoseYaw(){
    return m_PoseEstimator.GetEstimatedPosition().Rotation();
}





void subsystem_DriveTrain::ToggleBalanceCorrection(){
    m_IsBalancing = !m_IsBalancing;
}

void subsystem_DriveTrain::EnableBalanceCorrection(){
    m_IsBalancing = true;
}

void subsystem_DriveTrain::DisableBalanceCorrection(){
    m_IsBalancing = false;  
}

void subsystem_DriveTrain::ZeroGyro(){
    m_Gyro.SetYaw(0);
}

frc2::CommandPtr subsystem_DriveTrain::ZeroGyroCommand(){
  return RunOnce([this] { m_Gyro.SetYaw(0); });
}

void subsystem_DriveTrain::ImplementVisionPose(std::pair<frc::Pose2d, units::millisecond_t> pair){
    if(pair.second != units::second_t{-1.0}){
        frc::SmartDashboard::PutNumber("VisXPos", pair.first.X().value());
        frc::SmartDashboard::PutNumber("VisYPos", pair.first.Y().value());
        frc::SmartDashboard::PutNumber("VisRot", pair.first.Rotation().Degrees().value());
        if( ((units::math::fabs(pair.first.X() - GetPose().X()) < 1_m) && 
                (units::math::fabs( pair.first.Y() - GetPose().Y()) < 1_m)) || frc::DriverStation::IsDisabled()){
            m_PoseEstimator.AddVisionMeasurement(pair.first, pair.second);

        }
    }
}

void subsystem_DriveTrain::ResetModulesToAbsolute(){
    m_FrontLeftModule.ResetToAbsolute();
    m_FrontRightModule.ResetToAbsolute();
    m_BackLeftModule.ResetToAbsolute();
    m_BackRightModule.ResetToAbsolute();
}

units::meters_per_second_t subsystem_DriveTrain::AddPitchCorrection(){
    return units::meters_per_second_t{ m_PitchCorrectionPID.Calculate( m_Gyro.GetPitch() )};

}

units::meters_per_second_t subsystem_DriveTrain::AddRollCorrection(){
    return units::meters_per_second_t{ m_RollCorrectionPID.Calculate( m_Gyro.GetRoll() )};

}



void subsystem_DriveTrain::SetAutoOrient(bool IsOrientFront, bool IsOrientBack, double RotVelocity){
    
    if( (fabs(RotVelocity) <= ControllerConstants::Deadband)  && frc::DriverStation::IsTeleopEnabled()){      

        if( IsOrientFront ){
            if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
                m_Dpad =  DPAD::ORIENTATION::RIGHT;
            }else{
                m_Dpad =  DPAD::ORIENTATION::LEFT;
            }
            m_IsAutoOrient = true;
        }else if( IsOrientBack ){
            m_Dpad =  DPAD::ORIENTATION::DOWN;
            m_IsAutoOrient = true;
        }
        
    }else {
        m_IsAutoOrient = false;
        m_Dpad =  DPAD::ORIENTATION::NON_ORIENTED;
    }
}

units::radians_per_second_t subsystem_DriveTrain::GetAngularVelocity(){
    // double temp = m_Dpad;
    // m_ProfiledOrientPID.SetGoal(units::degree_t{temp + 0.0});

    // if(m_ProfiledOrientPID.AtSetpoint()){
    //     m_OrientCounter++;
    //     if(m_OrientCounter >= 4){
    //         m_IsAutoOrient = false;
    //         m_Dpad = DPAD::ORIENTATION::NON_ORIENTED;
    //         m_OrientCounter = 0;
    //         return 0_rad_per_s;
    //     }
    // }
    // return units::degrees_per_second_t{ m_ProfiledOrientPID.Calculate(m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees())};


    m_AutoOrientPID.SetSetpoint(m_Dpad);
    if(m_AutoOrientPID.AtSetpoint()){
        m_OrientCounter++;
        if(m_OrientCounter >= 4 ){
            m_IsAutoOrient = false;
            m_Dpad =  DPAD::ORIENTATION::NON_ORIENTED;
            m_OrientCounter = 0;
            return 0_rad_per_s;
        }
    }
    return units::radians_per_second_t{m_AutoOrientPID.Calculate(m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value() )};
    
}



void subsystem_DriveTrain::TogglePark(){
    m_IsPark  =  !m_IsPark;
}

bool subsystem_DriveTrain::IsPark(){
    return m_IsPark;
}





// std::pair<frc::Pose3d, units::time::second_t> subsystem_DriveTrain::getInitialPose()
// {

//   return estimator.Update();

// }

// int subsystem_DriveTrain::getID()
// {

//   return targetID;

// }

// bool subsystem_DriveTrain::HasAcceptableTargets(){
//   // frc::SmartDashboard::SmartDashboard::PutNumber("Size", (int)tags.size());
//   // frc::SmartDashboard::PutBoolean("resultTargets", result.HasTargets());
//   if(result.HasTargets()){


//     target = result.GetBestTarget();
//     targetID = target.GetFiducialId();   
//     frc::Transform3d camToTarget = target.GetBestCameraToTarget();

//     return  (target.GetPoseAmbiguity() <= 0.01) &&  ( target.GetFiducialId() > 0  && target.GetFiducialId() <= (int)tags.size() );

//   } else{
//     return false;
//   }
// }

// frc::DriverStation::Alliance subsystem_DriveTrain::GetAlliance(){
//   return m_Alliance;
// } 

// void subsystem_DriveTrain::SetAllianceOrigin(frc::DriverStation::Alliance AllianceColor){
//   if(m_Alliance == frc::DriverStation::Alliance::kRed){
//     aprilTags.get()->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
//   }else if(m_Alliance == frc::DriverStation::Alliance::kBlue){
//     aprilTags.get()->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
//   }
// }



// std::pair<frc::Pose2d, units::second_t> subsystem_DriveTrain::getEstimatedGlobalPose()
// {
//   if(HasAcceptableTargets()){
//     return std::make_pair<>(frc::Pose2d{0_m, 0_m, 0_rad}, units::second_t{2.7128183});
//   } else {
//   result = camera.GetLatestResult();
//   units::second_t timeStamp{result.GetTimestamp()};
//   target = result.GetBestTarget();
//   targetID = target.GetFiducialId();   
//   auto targetPose = aprilTags.get()->GetTagPose(targetID);
//   frc::Transform3d camToTarget = target.GetBestCameraToTarget();
//   frc::Pose3d camPose = targetPose.value().TransformBy(camToTarget.Inverse()); 
//   frc::Pose3d visionMeasurement = camPose.TransformBy(robotToCam.Inverse());
//   frc::SmartDashboard::PutNumber("VisionMesX", visionMeasurement.X().value());
//   frc::SmartDashboard::PutNumber("VisionMesY", visionMeasurement.Y().value());
//   frc::SmartDashboard::PutNumber("PoseAmbiguity", target.GetPoseAmbiguity());
//   frc::SmartDashboard::PutNumber("ToTarget", camToTarget.X().value());
//   frc::SmartDashboard::PutNumber("ID DETECTED", targetID);
//   // frc::SmartDashboard::PutNumber("ID DETECTED", targetID);
//   return std::make_pair<>(visionMeasurement.ToPose2d(), timeStamp);
//   // return std::make_pair(frc::Pose2d(), 0_ms);
//   }
// }






// This method will be called once per scheduler run
void subsystem_DriveTrain::Periodic() {

    frc::SmartDashboard::PutBoolean("AutoOrient", m_IsAutoOrient);
    frc::SmartDashboard::PutNumber("m_DPAD", m_Dpad);

    // frc::SmartDashboard::PutNumber("Pitch", m_Gyro.GetPitch());
    // frc::SmartDashboard::PutNumber("Roll", m_Gyro.GetRoll());
    // frc::SmartDashboard::PutNumber("PitchPID", AddPitchCorrection().value());
    // frc::SmartDashboard::PutNumber("RollPID", AddRollCorrection().value());
    frc::SmartDashboard::SmartDashboard::PutNumber("X Position", m_PoseEstimator.GetEstimatedPosition().X().value());
    frc::SmartDashboard::SmartDashboard::PutNumber("Y Position", m_PoseEstimator.GetEstimatedPosition().Y().value());
    frc::SmartDashboard::SmartDashboard::PutNumber("Pose Yaw", m_PoseEstimator.GetEstimatedPosition().Rotation().Degrees().value()); 

    //frc::SmartDashboard::SmartDashboard::PutNumber("Gyro Yaw (raw)", m_Gyro.GetYaw());
    //frc::SmartDashboard::SmartDashboard::PutNumber("GetYaw()", GetYaw().Degrees().value());

    //frc::SmartDashboard::PutBoolean("BalancedMethod", IsBalanced());
    // frc::SmartDashboard::PutBoolean("IsBalancing", m_IsBalancing);
    
    

    m_PoseEstimator.Update(m_Gyro.GetRotation2d(),
                      {m_FrontLeftModule.GetPosition(), 
                                m_FrontRightModule.GetPosition(),
                                m_BackLeftModule.GetPosition(),
                                m_BackRightModule.GetPosition()} );

   // frc::SmartDashboard::PutNumber("RoboMesX", m_PoseEstimator.GetEstimatedPosition().X().value());
   // frc::SmartDashboard::PutNumber("RoboMesY", m_PoseEstimator.GetEstimatedPosition().Y().value());
    
    // result = camera.GetLatestResult();


//   if(m_Chooser.GetSelected() != m_Alliance){
//     m_Alliance = m_Chooser.GetSelected();
//     SetAllianceOrigin(m_Alliance);
//   } 

//   frc::SmartDashboard::PutNumber("Alliance", frc::DriverStation::GetAlliance());
//   frc::SmartDashboard::PutNumber("Number", frc::DriverStation::GetLocation());

//   frc::SmartDashboard::PutBoolean("HasAcceptableTarget", HasAcceptableTargets());
//   if(HasAcceptableTargets()){
//     frc::Pose2d Pose = getEstimatedGlobalPose().first;
//     // frc::SmartDashboard::PutNumber("Vision X", Pose.X().value());
//     // frc::SmartDashboard::PutNumber("Vision Y", Pose.Y().value());
//   }
  // if(result.HasTargets()){
  //   units::second_t timeStamp{result.GetTimestamp()};
  //   target = result.GetBestTarget();
  //   targetID = target.GetFiducialId();   
  //   auto targetPose = aprilTags.get()->GetTagPose(targetID);
  //   frc::Transform3d camToTarget = target.GetBestCameraToTarget();
  //   frc::Pose3d camPose = targetPose.value().TransformBy(camToTarget.Inverse()); 
  //   frc::Pose3d visionMeasurement = camPose.TransformBy(robotToCam.Inverse());
  //   frc::SmartDashboard::PutNumber("VisionMesX", visionMeasurement.X().value());
  //   frc::SmartDashboard::PutNumber("VisionMesY", visionMeasurement.Y().value());  
    // frc::SmartDashboard::PutNumber( "PoseAmbiguity", target.GetPoseAmbiguity());
    // frc::SmartDashboard::PutNumber("ToTarget", camToTarget.X().value());
    // frc::SmartDashboard::PutNumber("ID DETECTED", targetID);

  // }

    // frc::SmartDashboard::PutBoolean("Balanced",IsBalanced());
    // frc::SmartDashboard::PutBoolean("IsBalancing?", m_IsBalancing);

    

}

