package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
@Deprecated
public class LeftHigh2AutonCommandSideways extends SequentialCommandGroup{
    public LeftHigh2AutonCommandSideways(MecDrivetrainSubsystem mecDrivetrainSubsystem, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        addCommands(    //Turn is Counterclockwise
//                new TurnToCommand(mecDrivetrainSubsystem, 90),
//                new TurnToCommand(mecDrivetrainSubsystem, 180),
//                new TurnToCommand(mecDrivetrainSubsystem, 270),
//                new TurnToCommand(mecDrivetrainSubsystem, 360),
//                new TurnToCommand(mecDrivetrainSubsystem, 0),

                /*new StrafeRightCommand(mecDrivetrainSubsystem, 68),
                new TurnToCommand(mecDrivetrainSubsystem, 36.5),
                new SlideHighBCommand(slide, arm, claw, turnServo,true),
                new WaitCommand(100),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem,-5.85),
                new WaitCommand(500),
                new DropConeCommand(claw, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem,6),
                new PrePick5FCommand(slide, claw, arm, turnServo),
                new TurnToCommand(mecDrivetrainSubsystem, 5),
                new DriveForwardCommand(mecDrivetrainSubsystem, 24.5),


                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw, turnServo,true),
                new TurnToCommand(mecDrivetrainSubsystem, 322, true),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -3),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(400),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 1.8),
                new PrePick5FCommand(slide, claw, arm,turnServo),
                new TurnToCommand(mecDrivetrainSubsystem, 3),
                new WaitCommand(200),



                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw, turnServo,true),
                new TurnToCommand(mecDrivetrainSubsystem, 322, true),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -2),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(400),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 1.8),
                new PrePick5FCommand(slide, claw, arm,turnServo),
                new TurnToCommand(mecDrivetrainSubsystem, 2),

                new WaitCommand(200),


                new PickCFCommand(slide, claw),
                new SlideLowBCommand(slide, arm, claw,turnServo, true),
                new TurnToCommand(mecDrivetrainSubsystem, 322),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -2),
                new DropAutoConeCommand(claw, slide, arm,true),
                new WaitCommand(400),
                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 1.8),
                new TurnToCommand(mecDrivetrainSubsystem, 8),
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
        new SlideResetUpAutonCommand(slide, arm, claw, turnServo)*/
        );
    }
}