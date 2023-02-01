// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_AlignToDesired.h"

command_AlignToDesired::command_AlignToDesired(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::function<double()> XDesired, std::function<double()> YDesired, std::function<double()> ThetaDesired): m_DriveTrain{DriveTrain}, m_PoseTracker{PoseTracker}, m_X{XDesired}, m_Y{YDesired}, m_Theta{ThetaDesired}{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  AddRequirements({m_PoseTracker});

  XPID.SetSetpoint(XDesired);
  YPID.SetSetpoint(YDesired);
  ThetaPID.SetSetpoint(ThetaDesired);

  

}

// Called when the command is initially scheduled.
void command_AlignToDesired::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_AlignToDesired::Execute() {

  if(m_PoseTracker->hasTarget()){
    m_DriveTrain->ImplementVisionPose(m_PoseTracker->getEstimatedGlobalPose());    
  }

  units::meters_per_second_t SupplementX { XPID.Calculate(m_DriveTrain->GetPose().X().value()) }; 
  units::meters_per_second_t SupplementY { YPID.Calculate(m_DriveTrain->GetPose().Y().value()) };
  units::degrees_per_second_t SupplementTheta { ThetaPID.Calculate(m_DriveTrain->GetPose().Rotation().Degrees().value()) };

  units::meters_per_second_t BaseSpeed{ 0 };
  units::degrees_per_second_t BaseAngularSpeed { 0 };


  m_DriveTrain->SwerveDrive(BaseSpeed + SupplementX, BaseSpeed + SupplementY, (BaseAngularSpeed + SupplementTheta), true, false);

}

// Called once the command ends or is interrupted.
void command_AlignToDesired::End(bool interrupted) {
  m_DriveTrain->SwerveDrive(0_mps, 0_mps,  0_rad_per_s, true, false);

}

// Returns true when the command should end.
bool command_AlignToDesired::IsFinished() {
  return false;
}
