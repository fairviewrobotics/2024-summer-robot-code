package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.SwerveUtils;

import java.util.function.DoubleSupplier;

public class RotateToSpeakerAndDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private Pose2d speakerPose;

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            VisionConstants.rotateToP,
            VisionConstants.rotateToI,
            VisionConstants.rotateToD,
            VisionConstants.rotateToConstraints
    );

    /**
     * Rotates the robot to face the speaker, while still allowing the driver to control forward and backward movement
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param forward The desired forward percentage of the robot
     * @param sideways The desired sideways percentage of the robot
     */
    public RotateToSpeakerAndDriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier sideways) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;

        rotationPID.setTolerance(0.05);

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        assert DriverStation.getAlliance().isPresent();

        this.speakerPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)? ShooterConstants.speakerPoseBlue : ShooterConstants.speakerPoseRed;

        addRequirements(swerveSubsystem);

    }

    @Override
    public void execute() {

        rotationPID.setGoal(SwerveUtils.rotateToPose(swerveSubsystem.getPose(), this.speakerPose) + Math.PI);


        //Calculated angle to rotate to
        //Rotation PID Calculations
        double odometryRotation = rotationPID.calculate(swerveSubsystem.getPose().getRotation().getRadians());

        swerveSubsystem.drive(
                -forward.getAsDouble(),
                -sideways.getAsDouble(),
                odometryRotation,
                true,
                true
        );

    }

    @Override
    public void initialize() {
        rotationPID.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }
}
