package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.arm.backside.auto.cone.ArmCone5BackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmLowFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.intake.AutoPickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnToCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

@Deprecated
public class RightHighJunctionCommandOld extends SequentialCommandGroup{
    public RightHighJunctionCommandOld(MecDrivetrainSubsystem mecDrivetrainSubsystem, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        /*
Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new ArmMidFrontCommand(slide, pivot, claw, turnServo, true),
                        new StrafeRightCommand(mecDrivetrainSubsystem, 52)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(mecDrivetrainSubsystem, 1),
                        new AutoDropConeCommand(claw, slide, pivot,true)
                        ),
                new ParallelCommandGroup(
                        new StrafeRightCommand(mecDrivetrainSubsystem, 17.8),
                        new ArmCone5BackCommand(slide, claw, pivot, turnServo)
                ),
                new DriveForwardCommand(mecDrivetrainSubsystem, -24.4),
//                new TurnToCommand(mecDrivetrainSubsystem, 0),



                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new ArmLowFrontCommand(slide, pivot, claw, turnServo, true),
                        new TurnToCommand(mecDrivetrainSubsystem, 61, true)
                ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(mecDrivetrainSubsystem, 3.47),
                        new WaitCommand(200),
                        new AutoDropConeCommand(claw, slide, pivot,true)
                        ),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(mecDrivetrainSubsystem, -1.77),
                        new ArmCone5BackCommand(slide, claw, pivot, turnServo)
                ),
                new TurnToCommand(mecDrivetrainSubsystem, 0),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(mecDrivetrainSubsystem, -2.6),
                        new InstantCommand(slide::slideCone4)
                ),


                new AutoPickConeCommand(slide, claw),
                new ParallelCommandGroup(
                        new TurnToCommand(mecDrivetrainSubsystem, 0),   //or remove
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true)
                ),
                new InstantCommand(pivot::moveInitializationPosition),

                new DriveForwardCommand(mecDrivetrainSubsystem, 28),
//                new StrafeLeftCommand(mecDrivetrainSubsystem, 1.51),
                new TurnToCommand(mecDrivetrainSubsystem, 271.2, true),
                new StrafeLeftCommand(mecDrivetrainSubsystem, 10.8),
                new WaitCommand(300),   //Just in case the pole is w0bbling



//                new SlowDriveForwardCommand(mecDrivetrainSubsystem, 0.85),
                new ParallelCommandGroup(
                        new SlowDriveForwardCommand(mecDrivetrainSubsystem, -1.43),
                        new AutoDropConeCommand(claw, slide, pivot,true)
                        ),

                new SlowDriveForwardCommand(mecDrivetrainSubsystem, -3),
                new ParallelCommandGroup(
                        new TurnToCommand(mecDrivetrainSubsystem, 271),
                        new SlideResetUpAutonCommand(slide, pivot, claw, turnServo)
                )
        );
    }
}