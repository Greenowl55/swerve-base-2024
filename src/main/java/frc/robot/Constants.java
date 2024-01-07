package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModule.SwerveModuleConfig;

public final class Constants {
    public static final class SwerveDriveConstants {
        public static final double MAX_MODULE_SPEED_METERS_PER_SECOND = 4;

        public static final double MAX_ROBOT_LINEAR_SPEED_METERS_PER_SECOND = 4;
        public static final double MAX_ROBOT_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;

        public static final SwerveModuleConfig FL_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig FR_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig RL_MODULE_CONFIG = new SwerveModuleConfig();
        public static final SwerveModuleConfig RR_MODULE_CONFIG = new SwerveModuleConfig();
        
        // Make sure to call this method to set your values before doing anything!
        public static void setupSwerveModuleConfigs() {
            // Front left module
            FL_MODULE_CONFIG.TURN_MOTOR_ID = 3;
            FL_MODULE_CONFIG.TURN_MOTOR_GEARING = 1 / 4.2;
            FL_MODULE_CONFIG.TURN_CONTROLLER_P = 5.0;
            FL_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            FL_MODULE_CONFIG.TURN_CONTROLLER_KS = 0.1;
            FL_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            FL_MODULE_CONFIG.TURN_INVERSE = false;
            FL_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            FL_MODULE_CONFIG.TURN_ENCODER_OFFSET = 42.0 / 360.0;
            FL_MODULE_CONFIG.TURN_ENCODER_ID = 3;

            FL_MODULE_CONFIG.DRIVE_MOTOR_ID = 4;
            FL_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 1 / 6.1;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_P = 0.5;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 1.25;
            FL_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 2.25;
            FL_MODULE_CONFIG.DRIVE_INVERSE = false;

            FL_MODULE_CONFIG.WHEEL_DIAMETER = 4.0;

            FL_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(11.0, 11.0);

            // Front right module
            FR_MODULE_CONFIG.TURN_MOTOR_ID = 7;
            FR_MODULE_CONFIG.TURN_MOTOR_GEARING = 1 / 4.2;
            FR_MODULE_CONFIG.TURN_CONTROLLER_P = 5.0;
            FR_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            FR_MODULE_CONFIG.TURN_CONTROLLER_KS = 0.1;
            FR_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            FR_MODULE_CONFIG.TURN_INVERSE = false;
            FR_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            FR_MODULE_CONFIG.TURN_ENCODER_OFFSET = 42.0 / 360.0;
            FR_MODULE_CONFIG.TURN_ENCODER_ID = 6;

            FR_MODULE_CONFIG.DRIVE_MOTOR_ID = 8;
            FR_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 1 / 6.1;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_P = 0.5;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 1.25;
            FR_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 2.25;
            FR_MODULE_CONFIG.DRIVE_INVERSE = false;

            FR_MODULE_CONFIG.WHEEL_DIAMETER = 4.0;

            FR_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(-11.0, 11.0);

            // Rear left module
            RL_MODULE_CONFIG.TURN_MOTOR_ID = 1;
            RL_MODULE_CONFIG.TURN_MOTOR_GEARING = 1 / 4.2;
            RL_MODULE_CONFIG.TURN_CONTROLLER_P = 5.0;
            RL_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            RL_MODULE_CONFIG.TURN_CONTROLLER_KS = 0.1;
            RL_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            RL_MODULE_CONFIG.TURN_INVERSE = false;
            RL_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            RL_MODULE_CONFIG.TURN_ENCODER_OFFSET = 42.0 / 360.0;
            RL_MODULE_CONFIG.TURN_ENCODER_ID = 9;

            RL_MODULE_CONFIG.DRIVE_MOTOR_ID = 2;
            RL_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 1 / 6.1;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_P = 0.5;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 1.25;
            RL_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 2.25;
            RL_MODULE_CONFIG.DRIVE_INVERSE = false;

            RL_MODULE_CONFIG.WHEEL_DIAMETER = 4.0;

            RL_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(11.0, -11.0);

            // Rear right module
            RR_MODULE_CONFIG.TURN_MOTOR_ID = 5;
            RR_MODULE_CONFIG.TURN_MOTOR_GEARING = 1 / 4.2;
            RR_MODULE_CONFIG.TURN_CONTROLLER_P = 5.0;
            RR_MODULE_CONFIG.TURN_CONTROLLER_D = 0;
            RR_MODULE_CONFIG.TURN_CONTROLLER_KS = 0.1;
            RR_MODULE_CONFIG.TURN_CONTROLLER_KV = 0;
            RR_MODULE_CONFIG.TURN_INVERSE = false;
            RR_MODULE_CONFIG.TURN_ENCODER_INVERSE = false;
            RR_MODULE_CONFIG.TURN_ENCODER_OFFSET = 42.0 / 360.0;
            RR_MODULE_CONFIG.TURN_ENCODER_ID = 12;

            RR_MODULE_CONFIG.DRIVE_MOTOR_ID = 6;
            RR_MODULE_CONFIG.DRIVE_MOTOR_GEARING = 1 / 6.1;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_P = 0.5;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_D = 0;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_KS = 1.25;
            RR_MODULE_CONFIG.DRIVE_CONTROLLER_KV = 2.25;
            RR_MODULE_CONFIG.DRIVE_INVERSE = false;

            RR_MODULE_CONFIG.WHEEL_DIAMETER = 4.0;

            RR_MODULE_CONFIG.MODULE_LOCATION = new Translation2d(-11.0, -11.0);
        }
    }
}
