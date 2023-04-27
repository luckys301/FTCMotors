package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone3BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone4BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone5BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnToCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

@Deprecated
public class RightHighJunctionCommand extends SequentialCommandGroup{
    public RightHighJunctionCommand(MecDrivetrainSubsystem mecDrivetrainSubsystem, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new ArmMidFrontCommand(slide, pivot, claw, turnServo, true),
                        new StrafeRightCommand(mecDrivetrainSubsystem, 52.51)
                ),
                new AutoDropConeCommand(claw, slide, pivot,true),
                new ParallelCommandGroup(
                        new StrafeRightCommand(mecDrivetrainSubsystem, 20.15),
                        new ArmCone5BackCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(mecDrivetrainSubsystem, -26.6),



                /***Cone 5***/
                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true),
                        new DriveForwardCommand(mecDrivetrainSubsystem, 29.9)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(mecDrivetrainSubsystem, -58.76),//61 ish
                        new DriveForwardCommand(mecDrivetrainSubsystem, 5.2),
                        new AutoDropConeCommand(claw, slide, pivot,true),
                        new DriveForwardCommand(mecDrivetrainSubsystem, -5)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(mecDrivetrainSubsystem, 1, true),
                        new ArmCone4BackCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(mecDrivetrainSubsystem, -31.7),



                /***Cone 4***/
                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true),
                        new DriveForwardCommand(mecDrivetrainSubsystem, 30.9)
                ),
                new SequentialCommandGroup(
                        new TurnCommand(mecDrivetrainSubsystem, -56.91),//oprg:300 to -60
                        new DriveForwardCommand(mecDrivetrainSubsystem, 5.1),
                        new AutoDropConeCommand(claw, slide, pivot,true),
                        new DriveForwardCommand(mecDrivetrainSubsystem, -3.7)
                ),
                new ParallelCommandGroup(
                        new TurnToCommand(mecDrivetrainSubsystem, 1.3, true),
                        new ArmCone3BackCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(mecDrivetrainSubsystem, -31.)



//                /***Cone 3***/
//                new PickConeCommand(slide, claw),
//                new ParallelCommandGroup(
//                        new SlideHighFCommand(slide, arm, claw, true),
//                        new DriveForwardCommand(mecDrivetrainSubsystem, 29.3)
//                ),
//                new TurnCommand(mecDrivetrainSubsystem, -53.5),//oprg:300 to -60
//                new DriveForwardCommand(mecDrivetrainSubsystem, 5.95),
//                new DropAutoConeCommand(claw, slide, arm,true),
//
//
//
//
//                //Parking
//                new ParallelCommandGroup(
//                        new SlideResetUpAutonCommand(slide, arm, claw),
//                        new TurnToCommand(mecDrivetrainSubsystem, 270)
//                ),
//                new DriveForwardCommand(mecDrivetrainSubsystem, -4)
//                new StrafeRightCommand(mecDrivetrainSubsystem, 40)
        );
    }
}