package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotorGroup;

@Config
public class Shooter extends SubsystemBase {
    public final PIDFController controller;
    public final NebulaMotorGroup motorGroup;

    public enum ShooterRPM {
        OUTTAKE(100),
        INTAKE(-100),
        STOP(0);

        public final double pivotPosition;
        ShooterRPM(double speed) {
            this.pivotPosition = speed;
        }
    }
    ShooterRPM shooterRPM = ShooterRPM.STOP;
    Telemetry telemetry;
    public final NebulaMotor motor, motor2;

    public Shooter(Telemetry tl, HardwareMap hw, Boolean isEnabled) {
        motor = new NebulaMotor(hw, NebulaConstants.Shooter.shooterMName,
            Motor.GoBILDA.RPM_312, NebulaConstants.Shooter.shooterDirection,
            NebulaMotor.IdleMode.Coast, isEnabled);
        motor2 = new NebulaMotor(hw, NebulaConstants.Shooter.shooterM2Name,
            Motor.GoBILDA.RPM_312, NebulaConstants.Shooter.shooter2Direction,
            NebulaMotor.IdleMode.Coast, isEnabled);
        motorGroup = new NebulaMotorGroup(motor, motor2);
//        motor.setDistancePerPulse(1);
        controller = new PIDFController(
            NebulaConstants.Pivot.pivotPID.p,
            NebulaConstants.Pivot.pivotPID.i,
            NebulaConstants.Pivot.pivotPID.d,
            NebulaConstants.Pivot.pivotPID.f,
            getShooterRPM(),
            getShooterRPM());
        controller.setTolerance(NebulaConstants.Pivot.pivotTolerance);
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        double output = (controller.calculate(getShooterRPM()));
        telemetry.addData("Motor RPM:", getShooterRPM());
        telemetry.addData("Motor Required RPM:", controller.getSetPoint());

        motorGroup.setPower(output);
    }


    public double getShooterRPM() {
        return motorGroup.getCorrectedVelocity();//TODO:Fix RPM Math
//        return 60 * ((double) motorGroup.getCorrectedVelocity() /
//            (double) Constants.SHOOTER_TPR);
    }
    // motor.getRPM() *60/motor.getCPR()
    // rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    //    ang_velocity_right = rpm_right * rpm_to_radians;
    //    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;

    public void setSetPoint(ShooterRPM pos) {
//        if(pos.pivotPosition>NebulaConstants.Pivot.MAX_POSITION ||
//            pos.pivotPosition<NebulaConstants.Pivot.MIN_POSITION){
//            motor.stopMotor();
//            return;
//        }
        controller.setSetPoint(pos.pivotPosition);
        shooterRPM = pos;
    }
    public void setSetPoint(double setPoint) {
//        if(setPoint>NebulaConstants.Pivot.MAX_POSITION ||
//            setPoint<NebulaConstants.Pivot.MIN_POSITION){
//            motor.stopMotor();
//            return;
//        }
        controller.setSetPoint(setPoint);
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint) {
        return new InstantCommand(()->{setSetPoint(setPoint);});
    }
    public Command setSetPointCommand(ShooterRPM pos) {
        return new InstantCommand(()->{setSetPoint(pos);});
    }

    public void encoderReset() {//Motors wouldn't need reset
        motorGroup.resetEncoder();
    }
    public double getSetPoint(){
        return controller.getSetPoint();
    }
}
