package org.firstinspires.ftc.teamcode.commands.drive.trajectory;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Trajectories;

public class DriveForwardCommand extends CommandBase{

    MecDriveSubsystem drive;
    double distance;
    Trajectory trajectory;
    MinVelocityConstraint constraint;
    public DriveForwardCommand(MecDriveSubsystem drive, double distance) {
        this.drive = drive;
        this.distance = distance;
        constraint = Trajectories.velConstraint;
        this.addRequirements(drive);
//        new DriveForwardCommand(drive, distance, Trajectories.velConstraint);
    }

    public DriveForwardCommand(MecDriveSubsystem drive, double distance, MinVelocityConstraint constraint) {
        this.drive = drive;
        this.distance = distance;
        this.constraint = constraint;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
//        if (distance < 0)
//            trajectory = new TrajectoryBuilder
//                    (drive.getPoseEstimate(), constraint, Trajectories.accelConstraint).back(-distance).build();
//        else
//            trajectory = new TrajectoryBuilder
//                    (drive.getPoseEstimate(), constraint, Trajectories.accelConstraint).forward(distance).build();

        if (distance < 0)
            trajectory = new TrajectoryBuilder
                    (PoseStorage.currentPose, constraint, Trajectories.accelConstraint).back(-distance).build();
        else
            trajectory = new TrajectoryBuilder
                    (PoseStorage.currentPose, constraint, Trajectories.accelConstraint).forward(distance).build();

        drive.followTrajectory(trajectory);

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
        PoseStorage.currentPose = trajectory.end();
        return !drive.isBusy();
    }
}