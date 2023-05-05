package org.firstinspires.ftc.teamcode.commands.drive.teleop.tank;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.sixWheelDrive.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

public class DefaultTankDriveCommand extends CommandBase {
    private final TankDriveSubsystem drive;
    private final GamepadEx driverGamepad;
    private static final PIDCoefficients X_TIPPING_PID = new PIDCoefficients(3, 0, 0);
    private static final PIDController xTipController = new PIDController(X_TIPPING_PID.kP, X_TIPPING_PID.kI, X_TIPPING_PID.kD);

    protected double multiplier;

    public DefaultTankDriveCommand(TankDriveSubsystem drive, GamepadEx driverGamepad) {
        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        double y = squareInput(driverGamepad.getLeftY()),
            x = squareInput(driverGamepad.getLeftX()),
            rx = squareInput(driverGamepad.getRightX());
        //TODO:See if this works
        if(Math.abs(drive.getDegreeRoll())> NebulaConstants.Drive.tippingTolerance){
            x = xTipController.calculate(drive.getDegreePitch(), 0);//Make sure this is the right IMU
        }
//        drive.arcadeDrive.arcadeDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightX() * multiplier);
        drive.tankDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightY() * multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
    private static double squareInput(double value) {
        return value * Math.abs(value);
    }
}
