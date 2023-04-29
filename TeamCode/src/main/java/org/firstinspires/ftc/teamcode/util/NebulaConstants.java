package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaServo;

//This will be used to store all Initialization Values for Subsystems, etc.
//Positions will
public class NebulaConstants {
    //Arm Distance Per Pulse
    //Math.PI/2
    //(Counts per revolution*Gear ratio(5:1=5 / 1:5))/(Turns per revolution * 2π)

    //Slide Distance Per Pulse
    //(COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);

    public Pivot pivot; //Most Likely Needed to View on Dashboard
    public static class Pivot {
        public static String pivotMName = "clawM";
        public NebulaMotor.Direction pivotDirection = NebulaMotor.Direction.Reverse;
//        public int pivotDistancePerPulse = 360 / (gear_ratio * pulses_per_rev);

        public static PIDFCoefficients pivotPID = new PIDFCoefficients(.005, 0.00, 0.0,0);
        public static int pivotTolerance = 10;
        public static double ks=0,
            kcos=0,
            ka=0,
            kv=0;
        public static double maxVelocity = 0,
            maxAcceleration = 0;
    }
    public Claw claw;
    public static class Claw {
        public static String clawSName = "clawS2";  //EH3
        public NebulaServo.Direction pivotDirection = NebulaServo.Direction.Reverse;
    }
    public Slide slide;
    public static class Slide {
        public static String slideMName1 = "lift";
        public static String slideMName2 = "lift2";
        public NebulaMotor.Direction slideDirection1 = NebulaMotor.Direction.Reverse,
            slideDirection2 = NebulaMotor.Direction.Forward;
        public static PIDFCoefficients slidePID = new PIDFCoefficients(0.005, 0.00, 0,0);//.0075, 0., .003, 0)
        public static int slideTolerance = 10;
//        public int slideDistancePerPulse = (COUNTS_PER_PULSE * GEAR_RATIO) / (GEAR_DIAMETER_INCHES * Math.PI);
    }
}
