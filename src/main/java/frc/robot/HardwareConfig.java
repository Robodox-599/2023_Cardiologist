
package frc.robot;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import frc.robot.Constants.SwerveConstants;

public class HardwareConfig {
    CANCoderConfiguration swerveCANCoderConfig;
    TalonFXConfiguration swerveAngleFXConfig;
    TalonFXConfiguration swerveDriveFXConfig;
    
    public HardwareConfig(){
        /* CANCoder Config */
        swerveCANCoderConfig = new CANCoderConfiguration();
        swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCANCoderConfig.sensorDirection = SwerveConstants.canCoderInvert;
        swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Drive Motor Config */
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig.slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(SwerveConstants.driveEnableCurrentLimit, 
                                                                                SwerveConstants.driveContinuousCurrentLimit,
                                                                                SwerveConstants.drivePeakCurrentLimit,
                                                                                SwerveConstants.drivePeakCurrentDuration);
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.voltageCompSaturation = SwerveConstants.kNominal;

        /* Angle Motor Config */
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveAngleFXConfig.slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveConstants.angleKD;
        swerveAngleFXConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(SwerveConstants.angleEnableCurrentLimit, 
                                                                                SwerveConstants.angleContinuousCurrentLimit,
                                                                                SwerveConstants.anglePeakCurrentLimit,
                                                                                SwerveConstants.anglePeakCurrentDuration);
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    }
}
