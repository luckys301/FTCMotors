
package org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;


public class MecDriveSubsystem extends SubsystemBase {

    private final MecDrive drive;
    private final int LFVal = 0,
            LRVal = 1,
            RFVal = 2,
            RRVal = 3;
    double[] powers = new double[4];



    public MecDriveSubsystem(MecDrive drive, Telemetry tl, HardwareMap hardwareMap) {
        this.drive = drive;
//        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        drive.update();
    }

    public void init() {
        drive.setMotorPowers(0, 0, 0, 0);
        setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0))); //SHOULD NOT BE DOING THIS, Autos never start in the center
        PoseStorage.currentPose = (new Pose2d(0, 0, Math.toRadians(0)));
    }

    public void reInitializeIMU() {
        drive.resetImu();
    }


    public void setPowers(double leftF, double leftR, double rightR, double rightF) {
        drive.setMotorPowers(leftF, leftR, rightR, rightF);
    }

    public void mecDrive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        powers [LFVal] = (y + x + rx) / denominator;    //fLPower
        powers [LRVal] = (y - x + rx) / denominator;    //bLPower
        powers [RFVal] = (y - x - rx) / denominator;    //fRPower
        powers [RRVal] = (y + x - rx) / denominator;    //bRPower
        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[RRVal]);
    }

    public void  fieldCentric(double y, double x, double rx){
//        double theta = -imu.getAngularOrientation().firstAngle;
        double theta = -drive.getExternalHeading();//Ok?

        double rotX = x * Math.cos(theta) - y * Math.sin(theta);
        double rotY = x * Math.sin(theta) + y * Math.cos(theta);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // ^^^^^^ Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        powers [LFVal] = (rotY + rotX - rx) / denominator;
        powers [LRVal] = (rotY - rotX - rx) / denominator;
        powers [RFVal] = (rotY + rotX + rx) / denominator;
        powers [RRVal] = (rotY - rotX + rx) / denominator;


        if(Math.abs(powers[LFVal])<0.25&Math.abs(powers[LRVal])<0.25&Math.abs(powers[RFVal])<0.25&Math.abs(powers[RRVal])<0.25){
            for (int i = 0; i <= 3; i++) {
//                powers[i] = squareInput(powers[i]);
                powers[i] = cubeInput(powers[i]);
            }
        }
        drive.setMotorPowers(powers[LFVal], powers[LRVal], powers[RFVal], powers[RRVal]);
    }

    private double squareInput(double power) {
        return power * Math.abs(power);
    }
    private double cubeInput(double power) {
        return power*Math.abs(power)*Math.abs(power);
    }

    public double getHeading() {//TODO: Does this make a difference
        return Math.toDegrees(drive.getExternalHeading());
//        return drive.getRawExternalHeading();
    }
    public double getHeadingDegrees() {//TODO: Does this make a difference
        return drive.getDegreeHeading();
    }





    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectoryBlock(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public void turnTo(double radians) {
        drive.turnAsync(radians);
    }

    public void turnBlock(double radians) {
        drive.turn(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Returns minimum range value if the given value is less than
     * the set minimum. If the value is greater than the set maximum,
     * then the method returns the maximum value.
     *
     * @param value The value to clip.
     */
    public double clipRange(double value) {
        return value <= -1 ? -1
                : value >= 1 ? 1
                : value;
    }

    /**
     * Normalize the wheel speeds
     */
    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if(maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }
    public void turnAsync(double angle) {
        drive.turnAsync(angle);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose);
    }


    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence);
    }

//    public int getRightAngle(){
//        return drive.getRightAngle();
//    }
//    public int getLeftAngle(){
//        return drive.getLeftAngle();
//    }
    public double getDegreePitch() {
        return drive.getDegreePitch();
    }
    public double getDegreeRoll() {
        return drive.getDegreeRoll();
    }

}