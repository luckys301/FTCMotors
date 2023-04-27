package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NebulaMotor {
    public enum IdleMode {
        Coast,
        Brake
    }
    private final MotorEx motor;
    private final Boolean isEnabled;
    public NebulaMotor(HardwareMap hM, String deviceId, Motor.GoBILDA type, Boolean isEnabled){
        motor = new MotorEx(hM, deviceId, type);
        this.isEnabled = isEnabled;
    }

    public void setPower(double power) {
        if (!isEnabled) motor.stopMotor();
            else motor.set(power);
    }

    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }
    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void stop() {
        motor.stopMotor();
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    public void setIdleMode(IdleMode behavior) {
        switch(behavior){
            case Coast: motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
                break;
            case Brake: motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                break;
        }
    }
}
