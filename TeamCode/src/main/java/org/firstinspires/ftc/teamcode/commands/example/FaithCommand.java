package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

public class FaithCommand extends SequentialCommandGroup{
    public FaithCommand(Slide slide, Pivot pivot, MecDrivetrainSubsystem mecDrivetrainSubsystem){
//        addRequirements(mecDrivetrainSubsystem);    //Add Subsystems that you need to run this Command
        addCommands(
                //Commands
        new InstantCommand(pivot::moveF, pivot),
//        new InstantCommand(slide::slideMid),
      new InstantCommand(slide::slideMid)   ,
//          new InstantCommand(mecDrivetrainSubsystem::)

                new DriveForwardCommand(mecDrivetrainSubsystem, 30)
        );
    }
}