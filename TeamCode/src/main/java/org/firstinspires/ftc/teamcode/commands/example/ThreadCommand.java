package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ThreadCommand extends SequentialCommandGroup{
    public ThreadCommand(MecDrivetrainSubsystem mecDrivetrainSubsystem, Pivot pivot, Slide slide, TurnServo turnServo){
        addRequirements(mecDrivetrainSubsystem);    //Add Subsystems that you need to run this Command - not necessary
        addCommands(
            new InstantCommand( //Better Idea to use Parallel Command
                () -> new Thread(() -> {
                    pivot.moveIntakeBAuto();
                    slide.slideResting();
                    turnServo.setFClawPos();
                }).start()
            )
        );
    }
}