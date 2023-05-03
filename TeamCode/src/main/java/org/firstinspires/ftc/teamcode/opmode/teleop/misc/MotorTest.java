package org.firstinspires.ftc.teamcode.opmode.teleop.misc;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;

//@Disabled
@TeleOp
public class MotorTest extends OpMode {
    NebulaMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(NebulaMotor.class, "motor");
        motor = new NebulaMotor(hardwareMap,
                "motorName",
                Motor.GoBILDA.RPM_84,
                NebulaMotor.Direction.Forward,
                NebulaMotor.IdleMode.Coast,
                true);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_down){motor.setPower(1);}
        else if(gamepad1.dpad_up){motor.setPower(-1);}
        else{motor.setPower(0);}
    }
}