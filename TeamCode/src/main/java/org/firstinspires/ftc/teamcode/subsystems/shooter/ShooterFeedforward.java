package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

@Config
public class ShooterFeedforward extends Shooter {
    public SimpleMotorFeedforward shooterFeedforward;

    TrapezoidProfile.State start = new TrapezoidProfile.State(getShooterRPM(), getShooterRPM());
    TrapezoidProfile.State goal;
    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile trapezoidProfile;
    Telemetry telemetry;
    public ShooterFeedforward(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        super(tl, hw, isEnabled);
        shooterFeedforward = new SimpleMotorFeedforward(
            NebulaConstants.Shooter.ks,
            NebulaConstants.Shooter.ka,
            NebulaConstants.Shooter.kv);
        constraints = new TrapezoidProfile.Constraints(
            0,// radians per second
            0);//radians per second per second

        trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
    }

    @Override
    public void periodic() {
//        slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));
        trapezoidProfile = new TrapezoidProfile(constraints, goal, start);
        start = trapezoidProfile.calculate(0.02);
        double output = (controller.calculate(getShooterRPM()) +
            (shooterFeedforward.calculate(start.position, start.velocity)));
        motorGroup.setPower(output);//TODO: Probably shouldn't be like this
        telemetry.addData("Motor RPM:", getShooterRPM());
        telemetry.addData("Motor Required RPM:", controller.getSetPoint());
    }
}