package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.logging.Level;

@Config
public class PivotFeedforward extends Pivot {
    //Zero HAS TO be parallel to the ground and encoder needs to be in radians
    private final static double POWER = 0.93;

    private ArmFeedforward armFeedforward;

    TrapezoidProfile.State start = new TrapezoidProfile.State(getEncoderDistance(), armMotor.getVelocity());
    TrapezoidProfile.State goal;
    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile trapezoidProfile;
    Telemetry telemetry;

    public PivotFeedforward(Telemetry tl, HardwareMap hw) {
        super(tl, hw);

        armFeedforward = new ArmFeedforward(NebulaConstants.Pivot.ks,NebulaConstants.Pivot.kcos, NebulaConstants.Pivot.ka, NebulaConstants.Pivot.kv);
        constraints = new TrapezoidProfile.Constraints(
            0,// radians per second
            0);//radians per second per second

        start = new TrapezoidProfile.State(getEncoderDistance(), armMotor.getVelocity());
        goal = new TrapezoidProfile.State(0,0);
        trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
    }


    public void periodic() {
        //Might need to make manual only feedforward
        if (armAutomatic) {
//            controller.setF(NebulaConstants.Pivot.pivotPID.f * Math.cos(Math.toRadians(controller.getSetPoint())));
            //^^^ Not needed in my opinion
            trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
            start = trapezoidProfile.calculate(0.02);
            double output = (controller.calculate(getEncoderDistance()) +
                (armFeedforward.calculate(start.position, start.velocity)));

            telemetry.addData("CLaw Motor Output:", output);
            telemetry.addData("Arm Velocity:", armMotor.getVelocity());

            armMotor.set(output);
        }
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Pos: ", pivotPos);

    }

    @Override
    public void setSetPoint(PivotPos pos) {
        super.setSetPoint(pos);
        start = new TrapezoidProfile.State(getEncoderDistance(), armMotor.getVelocity());
    }
    @Override
    public void setSetPoint(double setPoint, boolean shouldSensorWork) {
        super.setSetPoint(setPoint, shouldSensorWork);;
        start = new TrapezoidProfile.State(getEncoderDistance(), armMotor.getVelocity());
    }

    //TODO: Test!
    @Override
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        return new InstantCommand(()->{setSetPoint(setPoint, shouldSensorWork);});
    }
    @Override
    public Command setSetPointCommand(PivotPos pos) {
        return new InstantCommand(()->{setSetPoint(pos);});
    }
}
