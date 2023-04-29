package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

public class SelectCommand extends SequentialCommandGroup{
    public SelectCommand(MecDrivetrainSubsystem mecDrivetrainSubsystem){
        addRequirements(mecDrivetrainSubsystem);    //Add Subsystems that you need to run this Command - not necessary
        addCommands(
//            SelectCommand wobbleCommand = new SelectCommand(
//                    // the first parameter is a map of commands
//                    new HashMap<Object, Command>() {{
//                        put(Height.ZERO, new DriveForwardCommand(mecDrivetrainSubsystem,3));
//                        put(Height.ONE, new DriveForwardCommand(mecDrivetrainSubsystem,3));
//                        put(Height.FOUR, new DriveForwardCommand(mecDrivetrainSubsystem,3));
//                    }},
//                    // the selector
//                    this::height
//            )
        );
    }
}