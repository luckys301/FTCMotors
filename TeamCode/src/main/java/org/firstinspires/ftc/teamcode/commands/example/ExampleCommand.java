package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ExampleCommand extends SequentialCommandGroup{
    public ExampleCommand(MecDrivetrainSubsystem mecDrivetrainSubsystem){
        addRequirements(mecDrivetrainSubsystem);    //Add Subsystems that you need to run this Command
        addCommands(
        );
    }
}