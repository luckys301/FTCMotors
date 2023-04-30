package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

@Config
public class SlideFeedforward extends Slide {
    private ElevatorFeedforward slideFeedforward;

    TrapezoidProfile.State start = new TrapezoidProfile.State(getEncoderDistance(), slideM1.getVelocity());
    TrapezoidProfile.State goal;
    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile trapezoidProfile;
    Telemetry telemetry;
    public SlideFeedforward(Telemetry tl, HardwareMap hw) {
        super(tl, hw);
    }

    @Override
    public void periodic() {
        if (slideAutomatic) {
//            slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));
            slideFeedforward = new ElevatorFeedforward(
                NebulaConstants.Slide.ks,
                NebulaConstants.Slide.kcos,
                NebulaConstants.Slide.ka,
                NebulaConstants.Slide.kv);
            constraints = new TrapezoidProfile.Constraints(
                0,// radians per second
                0);//radians per second per second

            trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
            start = trapezoidProfile.calculate(0.02);
            double output = (slideController.calculate(getEncoderDistance()) +
                (slideFeedforward.calculate(start.position, start.velocity)));
            setPower(output);//TODO: Probably shouldn't be like this
        }
        telemetry.addLine("Slide - ");
        telemetry.addData("     Lift Motor Output:", output);
        telemetry.addData("     Lift1 Encoder: ", slideM1.getCurrentPosition());
        telemetry.addData("     List Pos:", getSetPoint());
    }

    @Override
    public void setSetPoint(LiftPos pos) {
        if(pos.liftPosition>NebulaConstants.Slide.MAX_POSITION ||
            pos.liftPosition<NebulaConstants.Slide.MIN_POSITION){
            slideM1.stopMotor();
            return;
        }
        slideController.setSetPoint(pos.liftPosition);
        liftPos = pos;
        this.lowBool = pos.lowBool;
    }
    @Override
    public void setSetPoint(double setPoint, boolean lowBool) {
        if(setPoint>NebulaConstants.Slide.MAX_POSITION ||
            setPoint<NebulaConstants.Slide.MIN_POSITION){
            slideM1.stopMotor();
            return;
        }        slideController.setSetPoint(setPoint);
        this.lowBool = lowBool;
    }

    //TODO: Test!
    @Override
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        return new InstantCommand(()->{this.setSetPoint(setPoint, shouldSensorWork);});
    }
    @Override
    public Command setSetPointCommand(LiftPos pos) {
        return new InstantCommand(()->{this.setSetPoint(pos);});
    }
}