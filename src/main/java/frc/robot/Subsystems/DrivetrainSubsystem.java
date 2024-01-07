package frc.robot.Subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveModule[] m_modules;
    private ChassisSpeeds m_speeds = new ChassisSpeeds();

    public DrivetrainSubsystem() {
        m_modules = new SwerveModule[] { 
            new SwerveModule(Constants.SwerveDriveConstants.FL_MODULE_CONFIG),
            new SwerveModule(Constants.SwerveDriveConstants.FR_MODULE_CONFIG),
            new SwerveModule(Constants.SwerveDriveConstants.RL_MODULE_CONFIG),
            new SwerveModule(Constants.SwerveDriveConstants.RR_MODULE_CONFIG),
        };
        m_kinematics = new SwerveDriveKinematics(m_modules[0].m_location, m_modules[1].m_location, m_modules[2].m_location, m_modules[3].m_location);
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setModuleState(states[i]);
            SmartDashboard.putNumber(i + "_Heading", m_modules[i].getTurnHeading());
            SmartDashboard.putNumber(i + "_Velocity", m_modules[i].getDriveVelocity());
        }
    }

    public void setChassisSpeeds(double x, double y, double omega) {
        m_speeds.vxMetersPerSecond = x;
        m_speeds.vyMetersPerSecond = y;
        m_speeds.omegaRadiansPerSecond = omega;
    }
}