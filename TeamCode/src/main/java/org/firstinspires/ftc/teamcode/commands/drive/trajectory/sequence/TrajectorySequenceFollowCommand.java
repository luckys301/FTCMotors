package org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class TrajectorySequenceFollowCommand extends CommandBase {
    private final MecDriveSubsystem drive;
    private final TrajectorySequence trajectorySequence;
    public TrajectorySequenceFollowCommand(MecDriveSubsystem drive, TrajectorySequence trajectorySequence) {
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        PoseStorage.currentPose = trajectorySequence.end();
        return !drive.isBusy();
    }
}