package org.firstinspires.ftc.teamcode.opmode.teleop.misc;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

//@Disabled
@TeleOp(group = "Subsystem test")
public class ClawServoTest extends OpMode {
    Servo servo2;
    double pos = Claw.ClawPos.OPEN_POS_S1.clawPosition, pos2 = Claw.ClawPos.CLOSE_POS_S1.clawPosition;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "clawS2");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        if(gamepad1.dpad_down){
            pos2 -= 0.001;
        }
        else if(gamepad1.dpad_up){
            pos2 += 0.001;
        }

        if(gamepad1.right_bumper){
//            pos = CLOSE_POS_S1;
            pos2  = Claw.ClawPos.CLOSE_POS_S1.clawPosition;
        }
        else if(gamepad1.left_bumper){
//            pos = OPEN_POS_S1;
            pos2  = Claw.ClawPos.OPEN_POS_S1.clawPosition;
        }

        pos2 = Math.min(Math.max(pos2, 0), 1);
        servo2.setPosition(pos2);
        telemetry.addData("Servo Pos: ",servo2.getPosition());
        telemetry.addData("Desired Pos: ", pos2);
        telemetry.update();

    }
}