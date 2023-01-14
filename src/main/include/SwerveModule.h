
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/MathUtil.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "Constants.h"
#include "HardwareConfig.h"
#include <units/angle.h>

class SwerveModule {
    public:
        SwerveModule(const double Module[]);
        void SetDesiredState(frc::SwerveModuleState& DesiredState, bool IsOpenLoop);
        frc::Rotation2d GetCANCoder();
        frc::SwerveModuleState Optimize(frc::SwerveModuleState DesiredState, frc::Rotation2d CurrentAngle);
        frc::SwerveModuleState GetState();
        void SetDegrees(units::degree_t Degrees);
        void SwapOrientation();   
        frc::SwerveModulePosition GetPosition();

        units::meter_t FalconToMeters(double Counts);    
        units::degree_t FalconToDegrees(double Counts);
        double DegreesToFalcon(units::degree_t Degrees);
        double FalconToRPM(double VelocityCounts);
        double RPMToFalcon(double RPM);
        double getTurnCounts();
        units::degree_t getLastAngle();
        units::meters_per_second_t FalconToMPS(double Velocitycounts);
        double MPSToFalcon(units::meters_per_second_t Velocity);
        

    private:
        void resetToAbsolute();
        
        units::degree_t m_LastAngle;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX m_DriveMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX m_AngleMotor;
        ctre::phoenix::sensors::WPI_CANCoder m_AngleEncoder;
        units::degree_t m_AngleOffset;

        frc::SimpleMotorFeedforward<units::feet> m_Feedforward;
        HardwareConfig m_Settings;




};