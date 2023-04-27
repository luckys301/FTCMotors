package org.firstinspires.ftc.teamcode.util.nebulaHardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class NebulaServo {
    private final SimpleServo servo;
    private final Boolean isEnabled;
    public NebulaServo(HardwareMap hM, String deviceId, double minAngle, double maxAngle, Boolean isEnabled){
        servo = new SimpleServo(hM, deviceId, minAngle, maxAngle, AngleUnit.DEGREES);
        this.isEnabled = isEnabled;
    }

    public void setPosition(double position) {
        if (!isEnabled) servo.setPosition(position);
            else servo.setPosition(servo.getPosition());
    }

    public void setInverted(boolean isInverted) {
        servo.setInverted(isInverted);
    }
    public double getPosition() {
        return servo.getPosition();
    }
    public double getAngle() {
        return servo.getAngle();
    }

    public void close() {
        servo.disable();
    }
    public void rotateByAngle(double angle) {
        servo.rotateByAngle(angle, AngleUnit.DEGREES);
    }
    public void turnToAngle(double angle) {
        servo.turnToAngle(angle, AngleUnit.DEGREES);
    }
}
