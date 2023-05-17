package org.firstinspires.ftc.teamcode.commands.drive.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Trajectories;

public class SlowSplineCommand extends CommandBase {
//    private static int num = 1;//BAD BAD BAD

    MecDriveSubsystem drive;
    Trajectory trajectory;
    boolean reverse = false;
    Vector2d splinePos;
    double endHeading;
    Pose2d poseToUse;

    MinVelocityConstraint maxVelConstraint;

    public SlowSplineCommand(MecDriveSubsystem drive, MinVelocityConstraint constraint, boolean reverse, Vector2d splinePos, double endHeading, Pose2d poseToUse) {
        this.drive = drive;
        this.reverse = reverse;
        this.splinePos = splinePos;
        this.endHeading = endHeading;
        this.maxVelConstraint = constraint;
        this.poseToUse = poseToUse;
        this.addRequirements(drive);

    }

    public SlowSplineCommand(MecDriveSubsystem drive, Vector2d splinePos, double endHeading) {
        this(drive, Trajectories.slowVelConstraint, false, splinePos, endHeading, PoseStorage.currentPose);
    }

    public SlowSplineCommand(MecDriveSubsystem drive, Vector2d splinePos, double endHeading, boolean reverse) {
        this(drive, Trajectories.slowVelConstraint, reverse, splinePos, endHeading, PoseStorage.currentPose);
    }

    public SlowSplineCommand(MecDriveSubsystem drive, Vector2d splinePos, double endHeading, Pose2d poseToUse) {
        this(drive, Trajectories.slowVelConstraint, false, splinePos, endHeading, poseToUse);
    }

    public SlowSplineCommand(MecDriveSubsystem drive, Vector2d splinePos, double endHeading, Pose2d poseToUse, boolean reverse) {
        this(drive, Trajectories.slowVelConstraint, reverse, splinePos, endHeading, poseToUse);
    }


    @Override
    public void initialize() {
//        trajectory = new TrajectoryBuilder
//                (drive.getPoseEstimate(), reverse, maxVelConstraint, Trajectories.accelConstraint)
//                .splineTo(splinePos, endHeading)
//                .build();

        trajectory = new TrajectoryBuilder
                (PoseStorage.currentPose, reverse, maxVelConstraint, Trajectories.accelConstraint)
                .splineTo(splinePos, endHeading)
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
//        new ResetPoseCommand(drive, splinePos, endHeading);
//        PoseStorage.currentPose = new Pose2d(splinePos.getX(), splinePos.getY(), endHeading);
        PoseStorage.currentPose = trajectory.end();

        return !drive.isBusy();
    }
}