package org.firstinspires.ftc.teamcode.opmode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive.MecDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
//@Disabled
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
//    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecDrive drive = new MecDrive(hardwareMap, telemetry, false);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(RoadrunnerValues.TurnTest.ANGLE));
    }
}
