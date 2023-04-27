package org.firstinspires.ftc.teamcode.opmode.auto.misc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.StickVision;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
@Disabled
@Autonomous
public class StickObserverOwnTest extends MatchOpMode {
    private MecDrivetrainSubsystem mecDrivetrainSubsystem;
    private StickVision stickVision;

    @Override
    public void robotInit() {
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
//        mecDrivetrainSubsystem.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

//        tagVision = new TagVision(hardwareMap, "Webcam 1", telemetry);
        stickVision = new StickVision(hardwareMap, telemetry, this);

//        singleServo = new SingleServo(hardwareMap, telemetry);
        while (!isStarted() && !isStopRequested())
        {
//            tagVision.updateTagOfInterest();
//            tagVision.tagToTelemetry();
//            stickVision.getTurning();
//            stickVision.periodic();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
//        tagNum = tagVision.getTag();
//        junctionVision = new JunctionVision(hardwareMap, telemetry);
        schedule(
//                new TurnCommand(mecDrivetrainSubsystem, stickVision.getTurning())

        );
//        if matchLoop();        stickVision.stopCamera();

    }
}