// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_AlignToDesired.h"

command_AlignToDesired::command_AlignToDesired(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::function<frc::Pose2d()> DesiredPose): m_DriveTrain{DriveTrain}, m_PoseTracker{PoseTracker}, m_DesiredPose{DesiredPose}{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_PoseTracker});
  AddRequirements({m_DriveTrain});

  m_ThetaProfiledPID.EnableContinuousInput(-180_deg, 180_deg);

  m_YProfiledPID.SetTolerance(0.1_m);
  m_XProfiledPID.SetTolerance(0.1_m);
  m_ThetaProfiledPID.SetTolerance(0.5_deg);


}

// Called when the command is initially scheduled.
void command_AlignToDesired::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_AlignToDesired::Execute() {

  // m_DriveTrain->ImplementVisionPose(m_DriveTrain->getEstimatedGlobalPose());
  // m_DriveTrain->ImplementVisionPose(m_PoseTracker->GetEstimatedGlobalPose());

  units::meters_per_second_t SupplementX { m_XProfiledPID.Calculate(m_DriveTrain->GetPose().X()) }; 
  units::meters_per_second_t SupplementY { m_YProfiledPID.Calculate(m_DriveTrain->GetPose().Y()) };
  units::degrees_per_second_t SupplementTheta { m_ThetaProfiledPID.Calculate(m_DriveTrain->GetPose().Rotation().Degrees()) };

  m_DriveTrain->SwerveDrive( SupplementX, SupplementY, SupplementTheta, true, false);

}

// Called once the command ends or is interrupted.
void command_AlignToDesired::End(bool interrupted) {
  m_DriveTrain->SwerveDrive(0_mps, 0_mps,  0_rad_per_s, true, false);
}

// Returns true when the command should end.
bool command_AlignToDesired::IsFinished() {
  return m_XProfiledPID.AtGoal() && m_YProfiledPID.AtGoal() && m_ThetaProfiledPID.AtGoal();
}
