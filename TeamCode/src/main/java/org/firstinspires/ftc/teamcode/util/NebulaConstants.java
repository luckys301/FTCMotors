package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaServo;

//This will be used to store all Initialization Values for Subsystems, etc.
//Positions will
public class NebulaConstants {
    public Pivot pivot; //Most Likely Needed to View on Dashboard
    public static class Pivot {
        public static String pivotMName = "clawM";
        public NebulaMotor.Direction pivotDirection = NebulaMotor.Direction.Reverse;
        public static PIDFCoefficients pivotPID = new PIDFCoefficients(.005, 0.00, 0.0,0);
        public static int controllerTolerance = 10;
    }
    public Claw claw;
    public static class Claw {
        public static String clawSName = "clawS2";  //EH3
        public NebulaServo.Direction pivotDirection = NebulaServo.Direction.Reverse;
    }
}
