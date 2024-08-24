package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTableUtils;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.VisionUtils;
import org.opencv.core.Mat;

import java.util.function.DoubleSupplier;

public class RotateToNoteAndDriveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private double rotateTo;
    private double seeNote;

    private Pose2d notePose;

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            VisionConstants.rotateToP,
            VisionConstants.rotateToI,
            VisionConstants.rotateToD,
            VisionConstants.rotateToConstraints
    );

    private final XboxController controller;

    private DoubleArrayEntry NTNotePose =  NetworkTableInstance.getDefault()
            .getTable("Debug").getDoubleArrayTopic("Pose").getEntry(new double[]{
        0.0,
               0.0,
    });

    private NetworkTableUtils NTUtils = new NetworkTableUtils("Debug");


    /**
     * Rotates the robot to face the speaker, while still allowing the driver to control forward and backward movement
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param controller The instance of {@link XboxController}
     * @param forward The desired forward percentage of the robot
     * @param sideways The desired sideways percentage of the robot
     * @param radians The desired rotation speed of the robot
     */
    public RotateToNoteAndDriveCommand(SwerveSubsystem swerveSubsystem, XboxController controller, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier radians, double noteTX) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;
        this.controller = controller;

        rotationPID.setTolerance(0.2);
        rotationPID.enableContinuousInput(-Math.PI*2, 0);

//        rotationPID.setGoal(0.0);

        addRequirements(swerveSubsystem);
    }


    @Override
    public void execute() {

//        tan theta = 0.03 / d
//

//        double cameraDist = 0.356 * (VisionConstants.FOCAL_LEN / VisionUtils.getThor());


        NTNotePose.set(new double[] {notePose.getX(), notePose.getY()});

        rotationPID.setGoal(SwerveUtils.rotateToPose(swerveSubsystem.getPose(), notePose));

        double angle = rotationPID.calculate(swerveSubsystem.getPose().getRotation().getRadians());
//       NTUtils.setDouble("Note Dist", dist);
////        System.out.println("Velocity Error: " + rotationPID.getVelocityError());
////        System.out.println("Position Error: " + rotationPID.getPositionError());
//
//        System.out.println("Dist: " + dist);
//        System.out.println("Cam Dist: " + cameraDist);

        if (seeNote == 1.0) {
            swerveSubsystem.drive(
                    forward.getAsDouble(),
                    sideways.getAsDouble(),
                    angle,
                    true,
                    true
            );
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        } else {
            swerveSubsystem.drive(
                    forward.getAsDouble(),
                    sideways.getAsDouble(),
                    radians.getAsDouble(),
                    true,
                    true
            );
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
        }
    }

    @Override
    public void initialize() {

        seeNote = VisionUtils.getNoteTV();

        double cameraDist = 0.35 / Math.tan(Math.toRadians(54 * (VisionUtils.getThor()/320)));
        double dist = Math.sqrt(Math.pow(cameraDist, 2) - Math.pow(VisionConstants.CAM_HEIGHT, 2));

        Pose2d robotPose = swerveSubsystem.getPose();

        notePose = new Pose2d(robotPose.getX() + Math.cos(robotPose.getRotation().getRadians() - Math.toRadians(VisionUtils.getNoteTX())) * dist,  robotPose.getY() + Math.sin(robotPose.getRotation().getRadians() - Math.toRadians(VisionUtils.getNoteTX())) * dist, new Rotation2d());

//        if (rotateTo < 0) {
//            rotateTo = Math.PI * 2 - rotateTo;
//        } else if (rotateTo > Math.PI * 2) {
//            rotateTo = rotateTo - Math.PI * 2;
//        }



    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
}
