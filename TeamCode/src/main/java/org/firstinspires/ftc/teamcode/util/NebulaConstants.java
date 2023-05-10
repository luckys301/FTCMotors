package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaServo;

//This will be used to store all Initialization Values for Subsystems, etc.
//Positions will
public class NebulaConstants {
    //TODO: Add IdleMode, and Motertype
    //Arm Distance Per Pulse
    //Math.PI/2
    //(Counts per revolution*Gear ratio(5:1=5 / 1:5))/(Turns per revolution * 2π)

    //Slide Distance Per Pulse
    //(COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);

    public Pivot pivot; //Most Likely Needed to View on Dashboard
    public static class Pivot {
        public static String pivotMName = "clawM";
        public static NebulaMotor.Direction pivotDirection = NebulaMotor.Direction.Reverse;
//        public int pivotDistancePerPulse = 360 / (gear_ratio * pulses_per_rev);// For Internal Encoder

        public static PIDFCoefficients pivotPID = new PIDFCoefficients(.005, 0.00, 0.0,0);
        public static int pivotTolerance = 10;
        public static double ks=0,
            kcos=0,
            ka=0,
            kv=0;
        public static double maxVelocity = 0,
            maxAcceleration = 0,
            MIN_POSITION = Math.toRadians(20),
            MAX_POSITION = Math.toRadians(230);
    }
    public Claw claw;
    public static class Claw {
        public static String clawSName = "clawS2";  //EH3
        public static NebulaServo.Direction pivotDirection = NebulaServo.Direction.Reverse;
    }
    public Slide slide;
    public static class Slide {
        public static String slideMName1 = "lift";
        public static String slideMName2 = "lift2";
        public static NebulaMotor.Direction slideDirection1 = NebulaMotor.Direction.Reverse,
            slideDirection2 = NebulaMotor.Direction.Forward;
        public static PIDFCoefficients slidePID = new PIDFCoefficients(0.005, 0.00, 0,0);//.0075, 0., .003, 0)
        public static int slideTolerance = 10;
//        public int slideDistancePerPulse = (COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);
//        public int slideDistancePerPulse = (GEAR_DIAMETER_INCHES * Math.PI);

        public static double ks=0,
            kcos=0,
            ka=0,
            kv=0;
        public static double maxVelocity = 0,
            maxAcceleration = 0,
            MIN_POSITION = 0,//mm
            MAX_POSITION = 0;
    }
    public Drive drive;
    public static class Drive {//TODO:FIX CONSTANTS
        public static String leftFrontM = "leftFront";
        public static String leftRearM = "leftRear";
        public static String rightRearM = "rightRear";
        public static String rightFrontM = "rightFront";
        public static NebulaMotor.Direction leftFrontDir = NebulaMotor.Direction.Reverse,
            leftRearDir = NebulaMotor.Direction.Reverse,
            rightRearDir = NebulaMotor.Direction.Forward,
            rightFrontDir = NebulaMotor.Direction.Forward;
        public static boolean isSquaredInputs = true;
        public static double tippingTolerance = 5;//This probably needs to be less
    }
    public ClawAlignToServo clawAlignToServo;
    public static class ClawAlignToServo {//14361
        public static String backL = "bL";
        public static String backR = "bR";
        public static String poleR = "pR";
        public static String poleL = "pL";
    }
    public static class Gamepad {
        public static double squareInput(double value) {
            return value * Math.abs(value);
        }
        public static double cubeInput(double value) {
            return value * Math.abs(value);
        }
        public static double isDriverOneDeadband(double value) {
            if(Math.abs(value)<0.1){
                return 0;
            } else return value;
        }
        public static double isDriverTwoDeadband(double value) {
            if(Math.abs(value)<0.1){
                return 0;
            } else return value;
        }
    }
    public Shooter shooter; //Most Likely Needed to View on Dashboard
    public static class Shooter {
        public static String shooterMName = "shooterM";
        public static String shooterM2Name = "shooterM2";
        public static NebulaMotor.Direction shooterDirection = NebulaMotor.Direction.Reverse;
        public static NebulaMotor.Direction shooter2Direction = NebulaMotor.Direction.Reverse;
//        public int pivotDistancePerPulse = 360 / (gear_ratio * pulses_per_rev);// For Internal Encoder

        public static PIDFCoefficients shooterPID = new PIDFCoefficients(.005, 0.00, 0.0,0);
        public static int shooterTolerance = 1;
        public static double ks=0,
            ka=0,
            kv=0;
        public static double maxVelocity = 0,
            maxAcceleration = 0;
    }
    public Hood hood; //Most Likely Needed to View on Dashboard
    public static class Hood {
        public static String shooterMName = "shooterM";
        public static String shooterM2Name = "shooterM2";
        public static NebulaMotor.Direction shooterDirection = NebulaMotor.Direction.Reverse;
        public static NebulaMotor.Direction shooter2Direction = NebulaMotor.Direction.Reverse;
//        public int pivotDistancePerPulse = 360 / (gear_ratio * pulses_per_rev);// For Internal Encoder

        public static PIDFCoefficients shooterPID = new PIDFCoefficients(.005, 0.00, 0.0,0);
        public static int shooterTolerance = 1;
        public static double ks=0,
            ka=0,
            kv=0;
        public static double maxVelocity = 0,
            maxAcceleration = 0;
    }
    public Intake intake; //Most Likely Needed to View on Dashboard
    public static class Intake {
        public static String intakeMName = "intakeM";
        public static String intakeM2Name = "intakeM2";
        public static NebulaMotor.Direction intakeDirection = NebulaMotor.Direction.Reverse;
        public static NebulaMotor.Direction intake2Direction = NebulaMotor.Direction.Reverse;
//        public int pivotDistancePerPulse = 360 / (gear_ratio * pulses_per_rev);// For Internal Encoder

        public static PIDFCoefficients intakePID = new PIDFCoefficients(.005, 0.00, 0.0,0);
        public static int intakeTolerance = 1;
        public static double ks=0,
            ka=0,
            kv=0;
        public static double maxVelocity = 0,
            maxAcceleration = 0;
        public static ElapsedTime intakeTime = new ElapsedTime(0);
    }
//From RobotAutoDriveByGyro_Linear.java
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//        (WHEEL_DIAMETER_INCHES * 3.1415);
}
