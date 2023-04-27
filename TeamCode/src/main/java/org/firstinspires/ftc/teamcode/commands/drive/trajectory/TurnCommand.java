package org.firstinspires.ftc.teamcode.commands.drive.trajectory;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

public class TurnCommand extends CommandBase {

    MecDrivetrainSubsystem drive;
    private final double angle;

    //Turns in a Counterclockwise direction
    public TurnCommand(MecDrivetrainSubsystem drive, double angle) {
        this.drive = drive;
        this.angle = angle;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.turn(Math.toRadians(angle));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}