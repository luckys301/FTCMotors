package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

@Deprecated
public class LeftHighAutonCommandSidewaysJUnctions extends SequentialCommandGroup{
    public LeftHighAutonCommandSidewaysJUnctions(MecDrivetrainSubsystem mecDrivetrainSubsystem, Slide slide, Pivot pivot, Claw claw){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(mecDrivetrainSubsystem, 90),
//                new TurnToCommand(mecDrivetrainSubsystem, 180),
//                new TurnToCommand(mecDrivetrainSubsystem, 270),
//                new TurnToCommand(mecDrivetrainSubsystem, 360),
//                new TurnToCommand(mecDrivetrainSubsystem, 0),

                /*new SlideMidBCommand(slide, arm, claw, turnServo, true),
                new StrafeRightCommand(mecDrivetrainSubsystem, 52.7),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -0.8),
                new DropAutoConeCommand(claw, slide, arm,true),
                new InstantCommand(claw::clawOpen),

                new WaitCommand(500),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new StrafeRightCommand(mecDrivetrainSubsystem, 16.5),
                new DriveForwardCommand(mecDrivetrainSubsystem, 26.5),


                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw, turnServo, true),
                new TurnCommand(mecDrivetrainSubsystem, -55.3),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -3),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(200),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 1.8),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new TurnToCommand(mecDrivetrainSubsystem, 3, true),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 3),
                new InstantCommand(slide::slideCone4),



                new PickCFCommand(slide, claw),
                new DriveForwardCommand(mecDrivetrainSubsystem, -35.8),
                new SlideHighFCommand(slide, arm, claw, true),
                new TurnToCommand(mecDrivetrainSubsystem, 264.9),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 2.1),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(300),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -2),

//                new TurnToCommand(mecDrivetrainSubsystem, 270),


//                new PickC3FCommand(slide, claw, arm, mecDrivetrainSubsystem),
//                new SlideLowAutonBCommand(slide, arm, claw),
//                new TurnToCommand(mecDrivetrainSubsystem, 322),
//                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -2),
//                new DropAutoConeCommand(claw, slide, arm),
//                new WaitCommand(400),
//                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 1.8),
//                new TurnToCommand(mecDrivetrainSubsystem, 8),
//                new PrePick2FCommand(slide, claw, arm),
//                new WaitCommand(200),






//                new PickC3BCommand(slide, claw, arm, mecDrivetrainSubsystem),
//                new TurnToCommand(mecDrivetrainSubsystem, 140),
//                new SlideLowFCommand(slide, arm, claw),
//                new InstantCommand(claw::clawOpen, claw),
//                new WaitCommand(600),
//                new TurnToCommand(mecDrivetrainSubsystem, 270),
//
//                new SlideIntakeFCommandT(slide, arm, claw),
//                new WaitCommand(200),
//                new InstantCommand(arm::moveReset, arm),
//                new DriveForwardCommand(mecDrivetrainSubsystem, 50)
                new SlideResetUpAutonCommand(slide, arm, claw),
                new StrafeRightCommand(mecDrivetrainSubsystem, 19)*/
        );
    }
}