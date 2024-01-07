package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class TeleOpCommand extends Command {
    private final DoubleSupplier m_xAxis;
    private final DoubleSupplier m_yAxis;
    private final DoubleSupplier m_omega;

    private final DrivetrainSubsystem m_drive;

    public TeleOpCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, DrivetrainSubsystem drive) {
        m_xAxis = x;
        m_yAxis = y;
        m_omega = omega;

        m_drive = drive;

        addRequirements(m_drive);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drive.setChassisSpeeds(
            m_xAxis.getAsDouble() * Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND,
            -m_yAxis.getAsDouble() * Constants.SwerveDriveConstants.MAX_MODULE_SPEED_METERS_PER_SECOND,
            -m_omega.getAsDouble() * Constants.SwerveDriveConstants.MAX_ROBOT_ANGULAR_SPEED_RADIANS_PER_SECOND
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}