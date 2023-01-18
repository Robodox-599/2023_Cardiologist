// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveAuton.h"

command_DriveAuton::command_DriveAuton(subsystem_DriveTrain* DriveTrain, std::string TrajFilePath, bool ToReset):
m_DriveTrain{DriveTrain}, m_ToReset{ToReset} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  // m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  m_Trajectory = pathplanner::PathPlanner::loadPath(TrajFilePath, pathplanner::PathConstraints(AutoConstants::MaxSpeed, AutoConstants::MaxAccel) );





}


// Called when the command is initially scheduled.
void command_DriveAuton::Initialize() {
  m_Timer.Stop();
  m_Timer.Reset();
  m_Timer.Start();

  if( m_ToReset ){
    m_DriveTrain->ResetOdometry(m_Trajectory.getInitialPose());
  }

    frc::ProfiledPIDController<units::angle::radian> ThetaPID{AutoConstants::AngleKP, 
                                              0, 
                                              AutoConstants::AngleKD, 
                                              frc::TrapezoidProfile<units::radians>::Constraints{AutoConstants::MaxAngularSpeed,
                                                                                                 AutoConstants::MaxAngularAccel}};

    ThetaPID.EnableContinuousInput(-1 * units::angle::radian_t{3.14}, units::angle::radian_t{3.14});


    
    frc::HolonomicDriveController m_DriveController{AutoConstants::XPID, AutoConstants::YPID, ThetaPID};
    pathplanner::PathPlannerTrajectory::PathPlannerState state = m_Trajectory.sample(m_Timer.Get()); 
    auto chasisSpeeds = m_DriveController.Calculate(m_DriveTrain->GetPose(), 
                                                    state.pose,
                                                    state.velocity,
                                                    state.holonomicRotation);
    auto ModuleStates = SwerveConstants::m_kinematics.ToSwerveModuleStates(chasisSpeeds);
    m_DriveTrain->SetModuleStates(ModuleStates);
}

// Called repeatedly when this Command is scheduled to run
void command_DriveAuton::Execute() {}

// Called once the command ends or is interrupted.
void command_DriveAuton::End(bool interrupted) {
  m_DriveTrain -> SwerveDrive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool command_DriveAuton::IsFinished() {
  return m_Timer.Get() >= (m_Trajectory.getTotalTime());
}
