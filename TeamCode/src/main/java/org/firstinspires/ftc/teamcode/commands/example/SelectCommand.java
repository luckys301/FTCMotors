package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

import java.util.HashMap;

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