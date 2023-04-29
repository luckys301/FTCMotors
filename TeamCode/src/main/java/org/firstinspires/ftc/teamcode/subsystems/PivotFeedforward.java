package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.logging.Level;

@Config
public class PivotFeedforward extends Pivot {
//private final PIDFController controller;
    private boolean armAutomatic;
//    public boolean shouldSensorWork = true;
//
//    Pivot.PivotPos pivotPos = Pivot.PivotPos.RESET;
//
    private final static double POWER = 0.93;
//    private double encoderOffset = 0;
    private ArmFeedforward armFeedforward;
//    private MotionProfile motionProfile;
//    MotionProfileBuilder motionProfileBuilder;
//    MotionProfileGenerator motionProfileGenerator;
//    MotionState start = new MotionState(0,0,0,0);
//    MotionState goal = new MotionState(0,0,0,0);

    Telemetry telemetry;
//    private final MotorEx armMotor;

    public PivotFeedforward(Telemetry tl, HardwareMap hw) {
        super(tl, hw);
        this.telemetry = tl;
        armFeedforward = new ArmFeedforward(NebulaConstants.Pivot.ks,NebulaConstants.Pivot.kcos, NebulaConstants.Pivot.ka, NebulaConstants.Pivot.kv);


//        motionProfileGenerator = new MotionProfileGenerator(start, goal, 0, 0, false);
//        motionProfileBuilder = new MotionProfileBuilder(start);
//        motionProfileGenerator = new MotionProfileGenerator();
    }

    @Override
    public void periodic() {
//        motionProfileBuilder = MotionProfileGenerator.generateSimpleMotionProfile(
//            new MotionProfileGenerator,
//            new MotionProfileBuilder,
//            NebulaConstants.Pivot.maxVelocity,
//            NebulaConstants.Pivot.maxAcceleration);
        if (armAutomatic) {

            controller.setF(NebulaConstants.Pivot.pivotPID.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            double output = (controller.calculate(getEncoderDistance()) +
                (armFeedforward.calculate(getEncoderDistance(),0)));
            //TODO: What would you put for Velocity
            telemetry.addData("CLaw Motor Output:", output);

            armMotor.set(output * POWER);
        }
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Pos: ", pivotPos);

    }
}
