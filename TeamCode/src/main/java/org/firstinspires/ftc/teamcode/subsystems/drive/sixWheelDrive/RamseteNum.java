package org.firstinspires.ftc.teamcode.subsystems.drive.sixWheelDrive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RamseteNum {
    //horizontal error adjustment
    public static double b = 12;
    public static double zeta = 1.3;

    //minimize overshoot
    public static double kLinear = 0;
    public static double kHeading = 4;

}