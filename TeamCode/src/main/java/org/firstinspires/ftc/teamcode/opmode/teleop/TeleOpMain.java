package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
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
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).and()
//            .whileHeld(new SlowDriveCommand(mecDriveSubsystem, driverGamepad, true));
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).or()
//            .whileHeld(new SlowDriveCommand(mecDriveSubsystem, driverGamepad, true));
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).and().negate()
//            .whileHeld(new SlowDriveCommand(mecDriveSubsystem, driverGamepad, true));
        mecDriveSubsystem.setDefaultCommand(new DefaultDriveCommand(mecDriveSubsystem, driverGamepad, true));
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
