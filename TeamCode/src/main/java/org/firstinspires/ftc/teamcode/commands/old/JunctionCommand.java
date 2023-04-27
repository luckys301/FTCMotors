package org.firstinspires.ftc.teamcode.commands.old;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.misc.JunctionVision;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.TeamMarkerPipeline;

import java.util.HashMap;
@Deprecated
public class JunctionCommand extends SequentialCommandGroup {
    public  JunctionCommand (JunctionVision junctionVision, MecDrivetrainSubsystem mecDrivetrainSubsystem){
        addCommands(
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(TeamMarkerPipeline.Position.LEFT, new SequentialCommandGroup(
                            //Low
                            new TurnCommand(mecDrivetrainSubsystem, -3)
//                            new SplineCommand(mecDrivetrainSubsystem, new Vector2d(21.5,25.8), Math.toRadians(47))
                            )
                    );
                    put(TeamMarkerPipeline.Position.MIDDLE, new SequentialCommandGroup(
                            //Mid
                            new TurnCommand(mecDrivetrainSubsystem, 0)
//                    new SplineCommand(mecDrivetrainSubsystem, new Vector2d(22.8,27.5), Math.toRadians(13))
                            )
                    );
                    put(TeamMarkerPipeline.Position.RIGHT, new SequentialCommandGroup(
                            //High
                            new TurnCommand(mecDrivetrainSubsystem, 3)
//                    new SplineCommand(mecDrivetrainSubsystem, new Vector2d(23.5,26), Math.toRadians(10))
                            )
                    );
                }}, junctionVision::getCurrentPosition)
        );
    }


}
