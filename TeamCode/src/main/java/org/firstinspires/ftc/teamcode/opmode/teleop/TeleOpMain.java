package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.commands.arm.manual.PivotMoveManual;
import org.firstinspires.ftc.teamcode.commands.arm.manual.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
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
    private MecDrivetrainSubsystem mecDrivetrainSubsystem;
    private Slide slide;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
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
//            .whileHeld(new SlowDriveCommand(mecDrivetrainSubsystem, driverGamepad, true));
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).or()
//            .whileHeld(new SlowDriveCommand(mecDrivetrainSubsystem, driverGamepad, true));
//        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).and().negate()
//            .whileHeld(new SlowDriveCommand(mecDrivetrainSubsystem, driverGamepad, true));
        mecDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(mecDrivetrainSubsystem, driverGamepad, true));
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
