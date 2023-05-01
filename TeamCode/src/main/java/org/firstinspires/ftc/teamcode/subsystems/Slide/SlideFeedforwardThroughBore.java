package org.firstinspires.ftc.teamcode.subsystems.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.odoPod.Encoder;

import java.util.Base64;

@Config
public class SlideFeedforwardThroughBore extends SlideFeedforward {
    private ElevatorFeedforward slideFeedforward;
    private Encoder slideEncoder;// Plugged into Motor;
    private double encoderOffset;

//    TrapezoidProfile.State start = new TrapezoidProfile.State(getEncoderDistance(), slideM1.getVelocity());
//    TrapezoidProfile.State goal;
//    TrapezoidProfile.Constraints constraints;
//    TrapezoidProfile trapezoidProfile;
//    Telemetry telemetry;
    public SlideFeedforwardThroughBore(Telemetry tl, HardwareMap hw) {
        super(tl, hw);
        slideEncoder = new Encoder(
            hw.get(DcMotorEx.class, NebulaConstants.Slide.slideMName1));
        slideEncoder.getCurrentPosition();
        slideEncoder.setDistancePerPulse(1);
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

//            trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
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
    public void resetEncoder() {
        encoderOffset = -(slideEncoder.getCurrentPosition()) + encoderOffset;
    }
    @Override
    public double getEncoderDistance() {
        return slideEncoder.getCurrentPosition() - encoderOffset;
    }
}