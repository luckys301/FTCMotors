package org.firstinspires.ftc.teamcode.commands.arm.backside;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmHighBackCommand extends SequentialCommandGroup {
    public ArmHighBackCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo, boolean auto){
        if (auto){
            addCommands(new InstantCommand(claw::clawClose),
                    new ParallelCommandGroup(
                            new InstantCommand(slide::slideHigh),
                            new InstantCommand(pivot::moveBAuto)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideHigh();
                                        pivot.moveHighB();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
    }
}
