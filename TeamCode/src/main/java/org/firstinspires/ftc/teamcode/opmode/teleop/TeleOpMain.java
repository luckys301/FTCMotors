package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.arm.manual.PivotMoveManual;
import org.firstinspires.ftc.teamcode.commands.arm.manual.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
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
    private TurnServo turnServo;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
        slide = new Slide(telemetry, hardwareMap);
//        pivot.resetOffset();
        pivot.moveInitializationPosition();
    }


    @Override
    public void configureButtons() {
        /*
         *  DRIVER
         */
        mecDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(mecDrivetrainSubsystem, driverGamepad, true));

        Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(mecDrivetrainSubsystem::reInitializeIMU));

        Button recenterIMU2 = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed(new InstantCommand(mecDrivetrainSubsystem::reInitializeIMU));

        Button slowMode = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new SlowDriveCommand(mecDrivetrainSubsystem, driverGamepad, true));

        /*
         * OPERATOR
         */

        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));

        pivot.setDefaultCommand(new PivotMoveManual(pivot, operatorGamepad::getLeftY));

        Button armIntake = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new SequentialCommandGroup());

        Button armOuttake = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new SequentialCommandGroup());

//        Button armGroundFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
//                .whenPressed(new ArmGroundFrontCommand(slide, pivot, claw, turnServo, false)));

        Button armLowFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SequentialCommandGroup()));

        Button armMidFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SequentialCommandGroup()));

        Button armHighFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SequentialCommandGroup()));

        Button armIntakeBack = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup()));

        Button pivotInitializationPosition = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(pivot::encoderReset));

        Button slideRecenter = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
                .whenPressed(slide::encoderRecenter);

//        Button pivotRecenter = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
//                .whenPressed(pivot::encoderReset);
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
