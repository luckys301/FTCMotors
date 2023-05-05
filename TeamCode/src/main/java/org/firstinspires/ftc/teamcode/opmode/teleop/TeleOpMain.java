package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.mec.DefaultMecDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.TurnToCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;
import org.firstinspires.ftc.teamcode.util.CycleTracker;
import org.firstinspires.ftc.teamcode.util.teleop.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {
    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private MecDriveSubsystem mecDriveSubsystem;
    private Slide slide;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        mecDriveSubsystem = new MecDriveSubsystem(new MecDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        mecDriveSubsystem.init();
        slide = new Slide(telemetry, hardwareMap);
//        pivot.resetOffset();
        pivot.moveInitializationPosition();
    }


    @Override
    public void configureButtons() {
        //Can Rumble Gamepads; If need to Rumble both Gamepads, might need to use queueEffect
        driverGamepad.gamepad.rumble(1,1, 100);

        //Ways to use buttons
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
            .and(new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)).negate()
            .whileActiveContinuous(new SequentialCommandGroup());
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
            .and(new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
            .whileActiveContinuous(new SequentialCommandGroup());

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)//To Test
            .whileActiveContinuous(new TurnToCommand(mecDriveSubsystem, 180+1e6));

        (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))//To Test
            .whileActiveOnce(new InstantCommand(()-> CycleTracker.trackCycle(1)));
        (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
            .whileActiveOnce(new InstantCommand(()-> CycleTracker.trackCycle(2)));
//        new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//            .and(new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER))
//            .whileActiveContinuous(new SlowMecDriveCommand(mecDriveSubsystem, driverGamepad, true));
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
//            .and(new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
//            .whileActiveContinuous(new SlowMecDriveCommand(mecDriveSubsystem, driverGamepad, true))
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).or()
//            .whileHeld(new SlowMecDriveCommand(mecDriveSubsystem, driverGamepad, true));
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).and().negate()
//            .whileHeld(new SlowMecDriveCommand(mecDriveSubsystem, driverGamepad, true));
        mecDriveSubsystem.setDefaultCommand(new DefaultMecDriveCommand(mecDriveSubsystem, driverGamepad, true));
    }

    @Override
    public void matchLoop() {}
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}
