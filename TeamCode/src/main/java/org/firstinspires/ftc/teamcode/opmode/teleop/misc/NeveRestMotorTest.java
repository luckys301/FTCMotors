package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.hardware.motors.NeveRest60Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide.Slide;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.Pivot;
import org.firstinspires.ftc.teamcode.util.teleop.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class NeveRestMotorTest extends MatchOpMode {
    // Gamepad
    private GamepadEx driverGamepad;
    private DcMotor neveRestM;
//    private NeveRest20Gearmotor neveRest20Gearmotor;
//    private NeveRest40Gearmotor neveRest40Gearmotor;
//    private NeveRest60Gearmotor neveRest60Gearmotor;
    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        neveRestM = hardwareMap.get(DcMotor.class, "neveRest");
        neveRestM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    @Override
    public void configureButtons() {
        new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
            .whileHeld(new InstantCommand(()-> {neveRestM.setPower(1);}))
            .whenReleased(new InstantCommand(()-> {neveRestM.setPower(0);}));
        new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whileHeld(new InstantCommand(()-> {neveRestM.setPower(-1);}))
            .whenReleased(new InstantCommand(()-> {neveRestM.setPower(0);}));
        ;
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
