package org.firstinspires.ftc.teamcode.opmode.auto.regional;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;

@Autonomous
//@Disabled
public class JustPark extends MatchOpMode {
    private int tagNum = 0;

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private MecDrivetrainSubsystem mecDrivetrainSubsystem;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;


    @Override
    public void robotInit() {
        claw = new Claw( telemetry, hardwareMap);
        pivot = new Pivot( telemetry, hardwareMap);
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
        slide = new Slide(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);

        tagVision = new TagVision(hardwareMap,  telemetry);
        while (!isStarted() && !isStopRequested())
        {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {tagNum = tagVision.getTag();

//        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(mecDrivetrainSubsystem, 20),
                                new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                new DriveForwardCommand(mecDrivetrainSubsystem, 12.25),
                                new StrafeLeftCommand(mecDrivetrainSubsystem, 29),
                                new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                new DriveForwardCommand(mecDrivetrainSubsystem, 6)
                        )
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(mecDrivetrainSubsystem, 35),
                                new SlideResetUpAutonCommand(slide, pivot, claw, turnServo)
                        )
                );
                return;
            }
            case 3:
            default: { //Right
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(mecDrivetrainSubsystem, 20),
                                new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                new DriveForwardCommand(mecDrivetrainSubsystem, 12.25),
                                new StrafeRightCommand(mecDrivetrainSubsystem, 29),
                                new DriveForwardCommand(mecDrivetrainSubsystem, 6)
                        )
                );
                return;
            }
        }
    }
}