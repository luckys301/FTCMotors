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
//    public enum Encoder {
//        Arm,
//        Slide,
//        None
//    }
    private final MotorEx motor;
    private final Boolean isEnabled;
//    private final Motor.GoBILDA type;
//    private int gearing;

    public NebulaMotor(HardwareMap hM, String deviceId, Motor.GoBILDA type, Direction direction, IdleMode behavior, Boolean isEnabled){
        motor = new MotorEx(hM, deviceId, type);
//        this.type = type;
        this.isEnabled = isEnabled;
//        this.gearing = gearing;

        switch (direction){ //Initialization of Motor Direction
            case Forward:
                setInverted(false);
                break;
            case Reverse:
                setInverted(true);
                break;
        }
        setIdleMode(behavior);
        motor.resetEncoder(); //Reset Encoder at the beginning of Initializatio
//        setDistancePerPulse(); //Do Seperately

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
    public double getVelocity() {
        return motor.getVelocity();
//        return motor.getCorrectedVelocity();
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
//    public void setDistancePerPulse(Encoder encoder){
//        //TODO: What should the conversion be to get distance in mm
//        //How would this change?
//        //The below was used previously
//        switch(encoder){
//            case Arm:
//                motor.setDistancePerPulse(360/(gearing*type.getCPR()));
//                //Math.PI/2
//                //(Counts per revolution*Gear ratio(5:1=5 / 1:5))/(Turns per revolution * 2π)
//                break;
//            case Slide:
//                motor.setDistancePerPulse((type.getCPR()*gearing)/ (_______*Math.PI));
//                //public static final double ROT_TO_INCHES = (COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);
//                break;
//            case None:
//                motor.setDistancePerPulse(1);
//                break;
//        }
//        motor.setDistancePerPulse(360/(gearing*type.getCPR()));
//    }
    public void setDistancePerPulse(int CPR){
        //TODO: What should the conversion be to get distance in mm
        //How would this change?
        //The below was used previously
        motor.setDistancePerPulse(360/CPR);
    }
}
