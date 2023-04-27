package org.firstinspires.ftc.teamcode.commands.old.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.AutoDropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

@Deprecated
public class JustONECone extends SequentialCommandGroup{
    public JustONECone(MecDrivetrainSubsystem mecDrivetrainSubsystem, Slide slide, Pivot pivot, Claw claw, TurnServo turnServo, SensorColor sensorColor){
        /*Turn is Counterclockwise*/
        addCommands(
                new ParallelCommandGroup(
                        new DriveForwardCommand(mecDrivetrainSubsystem, 53.91),
                        new ArmHighFrontCommand(slide, pivot, claw, turnServo, true)
                        ),
                new StrafeLeftCommand(mecDrivetrainSubsystem, 7.52),

                new AutoDropConeCommand(claw, slide, pivot, true),
                new WaitCommand(200),

                new SlideResetUpAutonCommand(slide, pivot, claw, turnServo)
                );
    }
}