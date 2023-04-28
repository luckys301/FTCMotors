package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
//List of Commands: https://docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system/convenience-commands
public class ExampleCommand extends SequentialCommandGroup{
    public ExampleCommand(Pivot pivot){
        addRequirements();    //Add Subsystems that you need to run this Command - not necessary
        addCommands(
//                new InstantCommand(claw::setFClawPos)
//                new InstantCommand(()->{pivot.moveIntakeBAuto();}) // This is how you use lamdas - Useful for commands that have parameters
        );
    }
}