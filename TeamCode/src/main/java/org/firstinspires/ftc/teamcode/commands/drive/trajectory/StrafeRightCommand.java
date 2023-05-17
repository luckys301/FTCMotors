package org.firstinspires.ftc.teamcode.commands.drive.trajectory;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Trajectories;

public class StrafeRightCommand extends CommandBase{

    MecDriveSubsystem drive;
    double distance;
    Trajectory trajectory;
    MinVelocityConstraint constraint;
    public StrafeRightCommand(MecDriveSubsystem drive, double distance) {
        this.drive = drive;
        this.distance = distance;
        constraint = Trajectories.velConstraint;
        this.addRequirements(drive);
    }

    public StrafeRightCommand(MecDriveSubsystem drive, double distance, MinVelocityConstraint constraint) {
        this.drive = drive;
        this.distance = distance;
        this.constraint = constraint;
        this.addRequirements(drive);
    }
    @Override
    public void initialize() {
//        trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .strafeRight(distance)
//                .build();
        trajectory = drive.trajectoryBuilder(PoseStorage.currentPose)
                .strafeRight(distance)
                .build();
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