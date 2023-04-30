package org.firstinspires.ftc.teamcode.util.nebulaHardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class NebulaServo {// Normal Servo
    public enum Direction {
        Forward, //False to Inverted
        Reverse //True to Inverted
    }
    private final SimpleServo servo;
    private final Boolean isEnabled;

    public NebulaServo(HardwareMap hM, String deviceId, Direction direction, double minAngle, double maxAngle, Boolean isEnabled){
        servo = new SimpleServo(hM, deviceId, minAngle, maxAngle, AngleUnit.DEGREES);
        this.isEnabled = isEnabled;
        switch (direction){ //Initialization of Motor Direction
            case Forward:
                setInverted(false);
                break;
            case Reverse:
                setInverted(true);
                break;
        }
    }

    public void setPosition(double position) {
        if (!isEnabled) servo.setPosition(servo.getPosition());//Instead of doing this, maybe close the servo
            else servo.setPosition(position);
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

    public void close() {//How does it disable, Restart Teleop or robot
        servo.disable();
    }
    public void rotateByAngle(double angle) {
        servo.rotateByAngle(angle, AngleUnit.DEGREES);
    }
    public void turnToAngle(double angle) {
        servo.turnToAngle(angle, AngleUnit.DEGREES);
    }
}
