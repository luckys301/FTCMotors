package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.old.auto.JustONECone;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;

@Autonomous(group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private MecDrivetrainSubsystem mecDrivetrainSubsystem;
    private Slide slide;
    private TurnServo turnServo;
    private SensorColor sensorColor;

//    public MecDrive mecanumDrive;


    @Override
    public void robotInit() {
        claw = new Claw( telemetry, hardwareMap);
        pivot = new Pivot(telemetry, hardwareMap);
        slide = new Slide(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        sensorColor = new SensorColor(hardwareMap, telemetry);
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
    }


    public void matchStart() {
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
                        new JustONECone(mecDrivetrainSubsystem, slide, pivot, claw, turnServo, sensorColor)
//                        new LeftStrafe(mecDrivetrainSubsystem, slide, arm, turnServo, sensorColor, claw)
//                        new RightSpline(mecDrivetrainSubsystem, slide, arm, claw)
//                      new LeftSplineValues(mecDrivetrainSubsystem, slide, arm, claw, turnServo, sensorColor)
//                        new Test(mecDrivetrainSubsystem, slide, arm, claw, turnServo)
                )
        );
//        PoseStorage.currentPose = mecDrivetrainSubsystem.getPoseEstimate();
    }


};