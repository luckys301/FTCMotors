package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.aprilTagPipeline.TagVision;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Autonomous
//@Disabled
public class AprilTagAutoImplementation extends MatchOpMode {
    private int tagNum = 0;

    private MecDrivetrainSubsystem mecDrivetrainSubsystem;
    private TagVision tagVision;


    @Override
    public void robotInit() {
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
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
        switch (tagNum) {
            case 1: { //Left
                schedule(
                        new SequentialCommandGroup(new DriveForwardCommand(mecDrivetrainSubsystem, 3))
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(new DriveForwardCommand(mecDrivetrainSubsystem, 7))
                );
                return;
            }
            case 3:
            default: { //Right
                schedule(
                        new SequentialCommandGroup()
                );
                return;
            }
        }
    }
}