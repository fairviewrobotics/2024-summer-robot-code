package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;



public class Rotate90 extends Command {

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
            0.3,
            0,
            0,
            new TrapezoidProfile.Constraints(Math.PI/4, Math.PI/8)
    );

    private double rotateTo;
    private final SwerveSubsystem swerveSubsystem;

    public Rotate90 (SwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        rotationPID.reset(swerveSubsystem.heading());
        this.swerveSubsystem = swerveSubsystem;
        rotationPID.setTolerance(0.0225);
        this.rotateTo = swerveSubsystem.heading() + Math.PI/2;
    }
    @Override
    public void execute () {
        System.out.println("Measurement: " + swerveSubsystem.heading());
        System.out.println("Goal: " + rotateTo);
        swerveSubsystem.drive(0, 0, rotationPID.calculate(swerveSubsystem.heading(), rotateTo), true, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, false, false);
    }
}
