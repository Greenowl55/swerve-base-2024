package frc.robot.util;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModule {
    public static class SwerveModuleConfig {
        public int TURN_MOTOR_ID;
        public double TURN_MOTOR_GEARING;
        public double TURN_CONTROLLER_P;
        public double TURN_CONTROLLER_D;
        public double TURN_CONTROLLER_KS;
        public double TURN_CONTROLLER_KV;
        public boolean TURN_INVERSE;
        public boolean TURN_ENCODER_INVERSE;
        public double TURN_ENCODER_OFFSET;
        public int TURN_ENCODER_ID;

        public int DRIVE_MOTOR_ID;
        public double DRIVE_MOTOR_GEARING;
        public double DRIVE_CONTROLLER_P;
        public double DRIVE_CONTROLLER_D;
        public double DRIVE_CONTROLLER_KS;
        public double DRIVE_CONTROLLER_KV;
        public boolean DRIVE_INVERSE;

        public double WHEEL_DIAMETER;

        public Translation2d MODULE_LOCATION = new Translation2d();
    }
    // CTRE
    // private final TalonFX m_turnCTRE;
    // private final TalonFXConfigurator m_turnConfigurator;
    // private final TalonFXConfiguration m_turnConfig;

    // private final TalonFX m_driveCTRE;
    // private final TalonFXConfigurator m_driveConfigurator;
    // private final TalonFXConfiguration m_driveConfig;
    // private final StatusSignal<Double> m_driveVelocity;
    // private final StatusSignal<Double> m_driveAcceleration;

    // private final CANcoder m_turnEncoderCTRE;
    // private final CANcoderConfigurator m_turnEncoderConfigurator;
    // private final CANcoderConfiguration m_turnEncoderConfiguration;
    // private final StatusSignal<Double> m_turnPositionSignal;
    // private final StatusSignal<Double> m_turnVelocitySignal;

    // private final VelocityVoltage m_velocityCommand;
    // private final PositionVoltage m_positionCommand;

    // REV
    private final CANSparkMax m_turnREV;
    private final SparkMaxAbsoluteEncoder m_turnEncoderREV;
    private final SparkMaxPIDController m_turnPIDController;
    private final CANSparkMax m_driveREV;
    private final SparkMaxPIDController m_drivePIDController;
    private final RelativeEncoder m_driveEncoderREV;

    public final Translation2d m_location;
    private final SwerveModuleConfig m_config;

    public SwerveModule(SwerveModuleConfig config) {
        // CTRE
        // m_turnCTRE = new TalonFX(config.TURN_MOTOR_ID);
        // m_turnConfigurator = m_turnCTRE.getConfigurator();
        // m_turnConfig = new TalonFXConfiguration();
        // m_turnConfig.MotorOutput.Inverted = config.TURN_INVERSE ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // m_turnConfig.Feedback.RotorToSensorRatio = config.TURN_MOTOR_GEARING;
        // m_turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // Change to RotorSensor if no CANcoder, or fused if using Phoenix Pro
        // m_turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        // m_turnConfig.Slot0.kP = config.TURN_CONTROLLER_P;
        // m_turnConfig.Slot0.kD = config.TURN_CONTROLLER_D;
        // m_turnConfig.Slot0.kS = config.TURN_CONTROLLER_KS;
        // m_turnConfig.Slot0.kV = config.TURN_CONTROLLER_KV;
        // m_turnConfigurator.apply(m_turnConfig);

        // m_driveCTRE = new TalonFX(config.DRIVE_MOTOR_ID);
        // m_driveConfigurator = m_driveCTRE.getConfigurator();
        // m_driveConfig = new TalonFXConfiguration();
        // m_driveConfig.MotorOutput.Inverted = config.DRIVE_INVERSE ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        // m_driveConfig.Feedback.SensorToMechanismRatio = config.DRIVE_MOTOR_GEARING;
        // m_driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // m_driveConfig.Slot0.kP = config.DRIVE_CONTROLLER_P;
        // m_driveConfig.Slot0.kD = config.DRIVE_CONTROLLER_D;
        // m_driveConfig.Slot0.kS = config.DRIVE_CONTROLLER_KS;
        // m_driveConfig.Slot0.kV = config.DRIVE_CONTROLLER_KV;
        // m_driveConfigurator.apply(m_driveConfig);
        // m_driveVelocity = m_driveCTRE.getVelocity();
        // m_driveAcceleration = m_driveCTRE.getAcceleration();

        // m_velocityCommand = new VelocityVoltage(0);
        // m_velocityCommand.Slot = 0;
        // m_positionCommand = new PositionVoltage(0);
        // m_positionCommand.Slot = 0;

        // CANCoder only
        // m_turnEncoderCTRE = new CANcoder(config.TURN_ENCODER_ID);
        // m_turnEncoderConfiguration = new CANcoderConfiguration();
        // m_turnEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // m_turnEncoderConfiguration.MagnetSensor.MagnetOffset = config.TURN_ENCODER_OFFSET;
        // m_turnEncoderConfiguration.MagnetSensor.SensorDirection = config.TURN_ENCODER_INVERSE ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // m_turnEncoderConfigurator = m_turnEncoderCTRE.getConfigurator();
        // m_turnEncoderConfigurator.apply(m_turnEncoderConfiguration);
        // m_turnPositionSignal = m_turnEncoderCTRE.getPosition();
        // m_turnVelocitySignal = m_turnEncoderCTRE.getVelocity();

        // REV
        m_turnREV = new CANSparkMax(config.TURN_MOTOR_ID, MotorType.kBrushless);
        m_turnREV.setInverted(config.TURN_INVERSE);
        m_turnEncoderREV = m_turnREV.getAbsoluteEncoder(Type.kDutyCycle);
        m_turnEncoderREV.setInverted(config.TURN_ENCODER_INVERSE);
        m_turnEncoderREV.setPositionConversionFactor(config.TURN_MOTOR_GEARING * 2 * Math.PI); // in rotations, we want radians
        m_turnEncoderREV.setVelocityConversionFactor(config.TURN_MOTOR_GEARING * 2 * Math.PI / 60); // In RPM, we want rad/sec
        m_turnEncoderREV.setZeroOffset(config.TURN_ENCODER_OFFSET);
        m_turnPIDController = m_turnREV.getPIDController();
        m_turnPIDController.setFeedbackDevice(m_turnEncoderREV);
        m_turnPIDController.setP(config.TURN_CONTROLLER_P);
        m_turnPIDController.setD(config.TURN_CONTROLLER_D);
        m_turnPIDController.setFF(config.TURN_CONTROLLER_KV);
        m_turnPIDController.setPositionPIDWrappingEnabled(true);
        m_turnPIDController.setPositionPIDWrappingMinInput(-Math.PI);
        m_turnPIDController.setPositionPIDWrappingMaxInput(Math.PI);


        m_driveREV = new CANSparkMax(config.DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_driveREV.setInverted(config.DRIVE_INVERSE);
        m_driveEncoderREV = m_driveREV.getEncoder();
        m_driveEncoderREV.setPositionConversionFactor(config.DRIVE_MOTOR_GEARING * Math.PI * config.WHEEL_DIAMETER); // in rotations, we want meters
        m_driveEncoderREV.setVelocityConversionFactor(config.DRIVE_MOTOR_GEARING * Math.PI * config.WHEEL_DIAMETER / 60); // In RPM, we want m/s
        m_drivePIDController = m_driveREV.getPIDController();
        m_drivePIDController.setFeedbackDevice(m_driveEncoderREV);
        m_drivePIDController.setP(config.DRIVE_CONTROLLER_P);
        m_drivePIDController.setD(config.DRIVE_CONTROLLER_D);
        m_drivePIDController.setFF(config.DRIVE_CONTROLLER_KV);

        m_location = config.MODULE_LOCATION;
        m_config = config;
    }

    public double getTurnHeading() {
        double rotations;
        double returnValue;

        // CTRE
        // BaseStatusSignal.refreshAll(m_turnPositionSignal, m_turnVelocitySignal);
        // rotations = BaseStatusSignal.getLatencyCompensatedValue(m_turnPositionSignal, m_turnVelocitySignal);
        // returnValue = rotations * 2 * Math.PI;

        // REV
        returnValue = m_turnEncoderREV.getPosition();
        
        return returnValue;
    }

    public double getDriveVelocity() {
        double velocity;
        double returnValue;

        // CTRE
        // BaseStatusSignal.refreshAll(m_driveVelocity, m_driveAcceleration);
        // velocity = BaseStatusSignal.getLatencyCompensatedValue(m_turnPositionSignal, m_turnVelocitySignal);
        // returnValue = velocity * Math.PI * m_config.WHEEL_DIAMETER;

        // REV
        returnValue = m_driveEncoderREV.getPosition();
        
        return returnValue;
    }

    public void setModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurnHeading()));

        // CTRE
        // m_velocityCommand.Velocity = state.speedMetersPerSecond / (Math.PI * m_config.WHEEL_DIAMETER);
        // m_positionCommand.Position = state.angle.getRotations();

        // m_driveCTRE.setControl(m_velocityCommand);
        // m_turnCTRE.setControl(m_positionCommand);

        // REV
        m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, Math.signum(state.speedMetersPerSecond) * m_config.DRIVE_CONTROLLER_KS);
        m_turnPIDController.setReference(state.angle.getRadians(), ControlType.kPosition, 0);
    }
}