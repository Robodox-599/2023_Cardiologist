#include "SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"

SwerveModule::SwerveModule(const double Module[] ):
                                                 m_DriveMotor{ (int)Module[0]},
                                                 m_AngleMotor{ (int)Module[1] },
                                                 m_AngleEncoder{ (int)Module[2] },
                                                 m_AngleOffset{ Module[3] },
                                                 m_Feedforward{SwerveConstants::DriveKS, SwerveConstants::DriveKV, SwerveConstants::DriveKA}
{
    //Config Angle Encoder
    m_AngleEncoder.ConfigFactoryDefault();
    m_AngleEncoder.ConfigAllSettings(m_Settings.SwerveCanCoderConfig);
    m_AngleEncoder.ConfigMagnetOffset(m_AngleOffset.value());
    //Config Angle Motor
    m_AngleMotor.ConfigFactoryDefault();
    m_AngleMotor.ConfigAllSettings(m_Settings.SwerveAngleFXConfig);
    m_AngleMotor.SetInverted(SwerveConstants::AngleMotorInvert);
    m_AngleMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
     m_AngleMotor.SetSelectedSensorPosition((DegreesToFalcon(0_deg - GetCANCoder().Degrees()) ));






    //Config Drive Motor
    m_DriveMotor.ConfigFactoryDefault();
    m_DriveMotor.ConfigAllSettings(m_Settings.SwerveDriveFXConfig);
    m_DriveMotor.SetInverted(SwerveConstants::DriveMotorInvert);
    m_DriveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_DriveMotor.SetSelectedSensorPosition(0);
    m_DriveMotor.EnableVoltageCompensation(true);

    m_LastAngle = GetState().angle.Degrees();
    
}



void SwerveModule::SetDegrees(units::degree_t Degrees){

    m_AngleMotor.SetSelectedSensorPosition( (DegreesToFalcon(Degrees - GetCANCoder().Degrees()) ));

}

double SwerveModule::getTurnCounts(){
    return m_AngleMotor.GetSelectedSensorPosition();
}
void SwerveModule::SwapOrientation(){
    m_DriveMotor.SetInverted(!m_DriveMotor.GetInverted());
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& DesiredState, bool IsOpenLoop){
    DesiredState = Optimize(DesiredState, GetState().angle);
    if(IsOpenLoop){
        double PercentOutput = DesiredState.speed / SwerveConstants::MaxSpeed;
        m_DriveMotor.Set(PercentOutput);
    }else{
        double Velocity = MPSToFalcon(DesiredState.speed);  
        double VoltageFeedForward = m_Feedforward.Calculate(DesiredState.speed)/SwerveConstants::kNominal;
        m_DriveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, Velocity, ctre::phoenix::motorcontrol::DemandType_ArbitraryFeedForward, VoltageFeedForward);
    }
    units::meters_per_second_t minSpeed = (SwerveConstants::MaxSpeed * 0.01);

    units::degree_t Angle = units::math::abs(DesiredState.speed) <= minSpeed ? m_LastAngle: DesiredState.angle.Degrees();

    m_AngleMotor.Set( ctre::phoenix::motorcontrol::ControlMode::Position, DegreesToFalcon(Angle) );
    m_LastAngle = Angle;
}

units::degree_t SwerveModule::getLastAngle(){
    return m_LastAngle;
}

/* This custom optimize method is created because Wpilib assumes the controller is continuous, which the CTRE Talons are not. */
frc::SwerveModuleState SwerveModule::Optimize(frc::SwerveModuleState DesiredState, frc::Rotation2d CurrentAngle){


    units::degree_t ModReferenceAngle { frc::AngleModulus( CurrentAngle.Radians() )  };
    frc::SmartDashboard::SmartDashboard::PutNumber("Current Angle", ModReferenceAngle.value());
    frc::SmartDashboard::SmartDashboard::PutNumber("Desired Angle(Continuous)", DesiredState.angle.Degrees().value());

    units::meters_per_second_t TargetSpeed = DesiredState.speed;
    units::degree_t Delta = DesiredState.angle.Degrees() - ModReferenceAngle;
    if(Delta >= 270_deg){
        Delta -= 360_deg;
    }else if(Delta <= -270_deg){
        Delta += 360_deg;
    }
    if( units::math::abs(Delta) > 90_deg){
        TargetSpeed = - TargetSpeed;
        Delta = Delta > 0_deg ? (Delta -= 180_deg) : ( Delta += 180_deg);
    }
    units::degree_t TargetAngle = CurrentAngle.Degrees() + Delta;
    frc::SmartDashboard::SmartDashboard::PutNumber("Desired Angle(Discontinuous)", DesiredState.angle.Degrees().value());

    return  {TargetSpeed, TargetAngle};
}


frc::Rotation2d SwerveModule::GetCANCoder(){
    return frc::Rotation2d( units::degree_t( m_AngleEncoder.GetAbsolutePosition() ) );
}

frc::SwerveModulePosition SwerveModule::GetPosition(){
    units::meter_t Distance{ FalconToMeters(m_DriveMotor.GetSelectedSensorPosition())};
    frc::Rotation2d Angle{FalconToDegrees( m_AngleMotor.GetSelectedSensorPosition()) };
    return {Distance, Angle};

}

units::meter_t SwerveModule::FalconToMeters(double Counts){
    return units::meter_t{ (Counts * SwerveConstants::WheelCircumference) / ( 2048 * SwerveConstants::DriveGearRatio)};
}

frc::SwerveModuleState SwerveModule::GetState(){
    units::meters_per_second_t Velocity{ FalconToMPS(m_DriveMotor.GetSelectedSensorVelocity(0)) };
    frc::Rotation2d Angle{FalconToDegrees( m_AngleMotor.GetSelectedSensorPosition()) };
    return {Velocity, Angle};
}

units::degree_t SwerveModule::FalconToDegrees(double Counts){
    return units::degree_t(Counts * ( 360.0 / (SwerveConstants::AngleGearRatio * 2048.0)));
}

double SwerveModule::DegreesToFalcon(units::degree_t Degrees){
    return Degrees.value() / (360.0 / (SwerveConstants::AngleGearRatio * 2048.0));
}

double SwerveModule::FalconToRPM(double VelocityCounts){
    double MotorRPM = VelocityCounts * ( 600.0 / 2048.0);
    double MechRPM = MotorRPM / SwerveConstants::DriveGearRatio;
    return MechRPM;
}

double SwerveModule::RPMToFalcon(double RPM){
    double MotorRPM = RPM * SwerveConstants::DriveGearRatio;
    double SensorCounts = MotorRPM * (2048.0 / 600.0);
    return SensorCounts;
}

units::meters_per_second_t SwerveModule::FalconToMPS(double VelocityCounts){
    double WheelRPM = FalconToRPM(VelocityCounts);
    units::meters_per_second_t WheelMPS = (WheelRPM * SwerveConstants::WheelCircumference) /60_s;
    return WheelMPS;
}

double SwerveModule::MPSToFalcon(units::meters_per_second_t Velocity){
    double WheelRPM = ( Velocity.value() * 60 ) / SwerveConstants::WheelCircumference.value();
    double WheelVelocity = RPMToFalcon(WheelRPM);
    return WheelVelocity;
}
