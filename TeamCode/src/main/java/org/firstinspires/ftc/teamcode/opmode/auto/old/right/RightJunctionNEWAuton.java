package org.firstinspires.ftc.teamcode.opmode.auto.old.right;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.old.auto.RightHighJunctionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
@Disabled
@Autonomous(name="rightNEW")
public class RightJunctionNEWAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
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
                                new RightHighJunctionCommand(mecDrivetrainSubsystem, slide, pivot, claw, turnServo),
                                /***Cone 3***/
                                new AutoPickConeCommand(slide, claw),
                                new DriveForwardCommand(mecDrivetrainSubsystem, 51),

//                                new ParallelCommandGroup(
//                                        new SlideHighFCommand(slide, arm, claw, true),
//                                        new DriveForwardCommand(mecDrivetrainSubsystem, 29.3)
//                                ),
//                                new TurnCommand(mecDrivetrainSubsystem, -52.47),//oprg:300 to -60
//                                new DriveForwardCommand(mecDrivetrainSubsystem, 5.95),
//                                new DropAutoConeCommand(claw, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                        new TurnToCommand(mecDrivetrainSubsystem, 270, true)
                                )
//                                new DriveForwardCommand(mecDrivetrainSubsystem, -4),
//
//                                new StrafeLeftCommand(mecDrivetrainSubsystem, 22.5)
                        )
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(
                                new RightHighJunctionCommand(mecDrivetrainSubsystem, slide, pivot, claw, turnServo),
                                /***Cone 3***/
                                new AutoPickConeCommand(slide, claw),
                                new ParallelCommandGroup(
                                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true),
                                        new DriveForwardCommand(mecDrivetrainSubsystem, 29.5)
                                ),
                                new TurnCommand(mecDrivetrainSubsystem, -54.8),//oprg:300 to -60
                                new DriveForwardCommand(mecDrivetrainSubsystem, 6),
                                new AutoDropConeCommand(claw, slide, pivot,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                        new TurnToCommand(mecDrivetrainSubsystem, 270)
                                ),
                                new DriveForwardCommand(mecDrivetrainSubsystem, -4),

                                new StrafeRightCommand(mecDrivetrainSubsystem, 12.7)
                        )
                );
                return;
            }
            default: { //High
                schedule(
                        new SequentialCommandGroup(
                                new RightHighJunctionCommand(mecDrivetrainSubsystem, slide, pivot, claw, turnServo),
                                new AutoPickConeCommand(slide, claw),
                                new WaitCommand(1000),

//                                /***Cone 3***/
//                                new PickConeCommand(slide, claw),
//                                new ParallelCommandGroup(
//                                        new SlideHighFCommand(slide, arm, claw, true),
//                                        new DriveForwardCommand(mecDrivetrainSubsystem, 29.3)
//                                ),
//                                new TurnCommand(mecDrivetrainSubsystem, -53.5),//oprg:300 to -60
//                                new DriveForwardCommand(mecDrivetrainSubsystem, 5.95),
//                                new DropAutoConeCommand(claw, slide, arm,true),




                                //Parking
                                new ParallelCommandGroup(
                                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo),
                                        new TurnToCommand(mecDrivetrainSubsystem, 270, true)
                                )
//                                new DriveForwardCommand(mecDrivetrainSubsystem, -4),
//                                new StrafeRightCommand(mecDrivetrainSubsystem, 41)
                        )
                );
                return;
            }
        }
    }
}