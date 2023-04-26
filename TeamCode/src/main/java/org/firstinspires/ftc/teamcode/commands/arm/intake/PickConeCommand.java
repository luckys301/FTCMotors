package org.firstinspires.ftc.teamcode.commands.arm.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class PickConeCommand extends SequentialCommandGroup {
    public PickConeCommand(Claw claw){
        addCommands(
                new InstantCommand(claw::clawClose, claw)
//                new WaitCommand(500),
//                new InstantCommand(slide::slidePickUp)
//                new InstantCommand(() ->
//                        new Thread(() -> {
//                            slide.slidePickUp();
//                            arm.moveReset();
//                        }).start())
        );
    }
}
