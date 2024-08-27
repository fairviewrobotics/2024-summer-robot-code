package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeOrShoot extends Command {
    public IndexerSubsystem indexerSubsystem;
    public IntakeSubsystem intakeSubsystem;

    private final Goal goal;

    private boolean gotNote = false;

    public AutoIntakeOrShoot(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem, Goal goal) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.goal = goal;
        addRequirements(indexerSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.isLimelight()) gotNote = true;
        switch (goal) {
            case INTAKE -> {
                if (!gotNote) {
                    intakeSubsystem.setSpeed(0.5);
                    indexerSubsystem.rotateAllWheelsPercent(0.2);
                } else  {
                    indexerSubsystem.rotateAllWheelsPercent(0);
                    intakeSubsystem.setSpeed(0.0);
                }
            }
            case SHOOT -> {
                intakeSubsystem.setSpeed(0.5);
                indexerSubsystem.rotateAllWheelsPercent(0.8);

            }
        }
    }

    @Override
    public void initialize() {
        gotNote = false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
        indexerSubsystem.rotateAllWheelsPercent(0);
    }
    public enum Goal {
        INTAKE,
        SHOOT
    }
}
