
package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
// import frc.robot.HardwareConfig;
import frc.robot.Constants.SwerveConstants;


public class SwerveModule {
    private WPI_TalonFX m_DriveMotor;
    private WPI_TalonFX m_AngleMotor;
    private WPI_CANCoder m_CANCoder;
    private double m_LastAngle;
    private double m_AngleOffset;
    private SimpleMotorFeedforward m_Feedforward;
    private HardwareConfig m_Settings;

    public SwerveModule(double[] module){
        m_DriveMotor = new WPI_TalonFX((int)module[0], "DriveCANivore");
        m_AngleMotor = new WPI_TalonFX((int)module[1], "DriveCANivore");
        m_CANCoder = new WPI_CANCoder((int)module[2], "DriveCANivore");
        m_AngleOffset = module[3];
        m_Feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
        m_Settings = new HardwareConfig();

        /* CANCoder Config */
        m_CANCoder.configFactoryDefault();
        m_CANCoder.configAllSettings(m_Settings.swerveCANCoderConfig);

        /* Angle Motor Config */
        m_AngleMotor.configFactoryDefault();
        m_AngleMotor.configAllSettings(m_Settings.swerveAngleFXConfig);
        m_AngleMotor.setInverted(SwerveConstants.angleMotorInvert);
        m_AngleMotor.setNeutralMode(NeutralMode.Coast);
        m_AngleMotor.setSelectedSensorPosition(degreesToFalcon(getCANCoder().getDegrees() - m_AngleOffset));

        /* Drive Motor Config */
        m_DriveMotor.configFactoryDefault();
        m_DriveMotor.configAllSettings(m_Settings.swerveDriveFXConfig);
        m_DriveMotor.setInverted(SwerveConstants.driveMotorInvert);
        m_DriveMotor.setNeutralMode(NeutralMode.Brake);
        m_DriveMotor.setSelectedSensorPosition(0.0);
        m_DriveMotor.enableVoltageCompensation(true);

        m_LastAngle = getState().angle.getDegrees();
    }

    public double getTurnCounts(){
        return m_AngleMotor.getSelectedSensorPosition();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = optimize(desiredState, getState().angle);
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            m_DriveMotor.set(percentOutput);
        } else {
            double velocity = mpsToFalcon(desiredState.speedMetersPerSecond);
            double voltageFeedForward = m_Feedforward.calculate(desiredState.speedMetersPerSecond) / SwerveConstants.kNominal;
            m_DriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, voltageFeedForward);
        }
        double minSpeed = SwerveConstants.maxSpeed * 0.01;
        double angle = Math.abs(desiredState.speedMetersPerSecond) <= minSpeed ? m_LastAngle : desiredState.angle.getDegrees();
        m_AngleMotor.set(ControlMode.Position, degreesToFalcon(angle));
        m_LastAngle = angle;
    }

    public void setDesiredAngle(Rotation2d angle){
        SwerveModuleState tempState = new SwerveModuleState(0.0, angle);
        tempState = optimize(tempState, getState().angle);
        m_AngleMotor.set(ControlMode.Position, degreesToFalcon(angle.getDegrees()));
        m_LastAngle = angle.getDegrees();
    }

    public double getLastAngle(){
        return m_LastAngle;
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double modReferenceAngle = MathUtil.angleModulus(currentAngle.getRadians()) * 180.0 / Math.PI;
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = desiredState.angle.getDegrees() - modReferenceAngle;
        if(delta >= 270.0){
            delta -= 360.0;
        }
        if(delta <= -270.0){
            delta += 360.0;
        }
        if(Math.abs(delta) > 90.0){
            targetSpeed *= -1.0;
            delta = delta > 0.0 ? delta - 180.0 : delta + 180.0;
        }
        double targetAngle = currentAngle.getDegrees() + delta;

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    public void resetToAbsolute(){
        m_AngleMotor.setSelectedSensorPosition(degreesToFalcon(getCANCoder().getDegrees() - m_AngleOffset));
    }

    public Rotation2d getCANCoder(){
        return Rotation2d.fromDegrees(m_CANCoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition(){
        double distance = falconToMeters(m_DriveMotor.getSelectedSensorPosition());
        Rotation2d angle = Rotation2d.fromDegrees(falconToDegrees(m_AngleMotor.getSelectedSensorPosition()));
        return new SwerveModulePosition(distance, angle);
    }

    public SwerveModuleState getState(){
        double velocity = falconToMPS(m_DriveMotor.getSelectedSensorVelocity(0));
        Rotation2d angle = Rotation2d.fromDegrees(falconToDegrees(m_AngleMotor.getSelectedSensorPosition()));
        return new SwerveModuleState(velocity, angle);
    }

    public double falconToMeters(double counts){
        return (counts * SwerveConstants.wheelCircumference) / (2048.0 * SwerveConstants.driveGearRatio);
    }

    public double falconToDegrees(double counts){
        return counts * (360.0 / (SwerveConstants.angleGearRatio * 2048.0));
    }

    public double degreesToFalcon(double degrees){
        return degrees / (360.0 / (SwerveConstants.angleGearRatio * 2048.0));
    }

    public double falconToRPM(double velocityCounts){
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        return motorRPM / SwerveConstants.driveGearRatio;
    }

    public double rpmToFalcon(double rpm){
        double motorRPM = rpm * SwerveConstants.driveGearRatio;
        return motorRPM * (2048.0 / 600.0);
    }

    public double falconToMPS(double velocityCounts){
        double wheelRPM = falconToRPM(velocityCounts);
        return wheelRPM * SwerveConstants.wheelCircumference / 60.0;
    }

    public double mpsToFalcon(double mps){
        double wheelRPM = (mps * 60.0) / SwerveConstants.wheelCircumference;
        return rpmToFalcon(wheelRPM);
    }
}
