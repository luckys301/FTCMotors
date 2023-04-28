package org.firstinspires.ftc.teamcode.util.nebulaHardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NebulaMotor {
    public enum IdleMode {
        Coast,
        Brake
    }
    public enum Direction {
        Forward, //False to Inverted
        Reverse //True to Inverted
    }
    private final MotorEx motor;
    private final Boolean isEnabled;
    private final Motor.GoBILDA type;
    public NebulaMotor(HardwareMap hM, String deviceId, Motor.GoBILDA type, Direction direction, Motor.ZeroPowerBehavior behavior, Boolean isEnabled){
        motor = new MotorEx(hM, deviceId, type);
        this.type = type;
        this.isEnabled = isEnabled;
        switch (direction){ //Initialization of Motor Direction
            case Forward:
                setInverted(false);
                break;
            case Reverse:
                setInverted(true);
                break;
        }
        motor.setZeroPowerBehavior(behavior);
        motor.resetEncoder(); //Reset Encoder at the beginning of Initializatio
        setDistancePerPulse(); //TODO: Check which one to do

        setPower(0); //Might be unnecessary - no risk in leaving it
    }

    public void setPower(double power) {
        if (!isEnabled) motor.stopMotor();
            else motor.set(power);
    }

    private void setInverted(boolean isInverted) {
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
    public void setDistancePerPulse(){
        //TODO: What should the conversion be to get distance in mm
        //How would this change?
        //The below was used previously
        motor.setDistancePerPulse(360/type.getCPR());
    }
    public void setDistancePerPulse(int CPR){
        //TODO: What should the conversion be to get distance in mm
        //How would this change?
        //The below was used previously
        motor.setDistancePerPulse(360/CPR);
    }
}
