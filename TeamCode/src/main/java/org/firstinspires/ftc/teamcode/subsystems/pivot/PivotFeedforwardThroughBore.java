package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.odoPod.Encoder;

import java.util.logging.Level;

@Config
public class PivotFeedforwardThroughBore extends PivotFeedforward {
//    private ElevatorFeedforward slideFeedforward;
    protected final Encoder armEncoder;// Plugged into Motor;
//    private double encoderOffset;

//    TrapezoidProfile.State start = new TrapezoidProfile.State(getEncoderDistance(), slideM1.getVelocity());
//    TrapezoidProfile.State goal;
//    TrapezoidProfile.Constraints constraints;
//    TrapezoidProfile trapezoidProfile;
//    Telemetry telemetry;
    public PivotFeedforwardThroughBore(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        super(tl, hw, isEnabled);
        armEncoder = new Encoder(
            hw.get(DcMotorEx.class, NebulaConstants.Pivot.pivotMName));
        armEncoder.getCurrentPosition();
        armEncoder.setDistancePerPulse(1);
    }

    @Override
    public void periodic() {
        if (armAutomatic) {
//            slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));
            constraints = new TrapezoidProfile.Constraints(
                0,// radians per second
                0);//radians per second per second

//            trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
            start = trapezoidProfile.calculate(0.02);
            double output = (controller.calculate(getEncoderDistance()) +
                (armFeedforward.calculate(start.position, start.velocity)));
            armMotor.setPower(output);//TODO: Probably shouldn't be like this
        }
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Pos: ", pivotPos);
    }

//    @Override
//    public void resetEncoder() {
//        encoderOffset = -(slideEncoder.getCurrentPosition()) + encoderOffset;
//    }
    @Override
    public double getEncoderDistance() {
        return armEncoder.getCurrentPosition() - encoderOffset;
    }
}