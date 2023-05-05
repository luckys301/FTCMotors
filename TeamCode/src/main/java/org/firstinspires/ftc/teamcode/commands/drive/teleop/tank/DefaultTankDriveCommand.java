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
    private static final PIDCoefficients Y_TIPPING_PID = new PIDCoefficients(3, 0, 0);
    private static final PIDController yTipController = new PIDController(Y_TIPPING_PID.kP, Y_TIPPING_PID.kI, Y_TIPPING_PID.kD);

    protected double multiplier;

    public DefaultTankDriveCommand(TankDriveSubsystem drive, GamepadEx driverGamepad) {
        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        double y = NebulaConstants.Gamepad.isDriverOneDeadband(driverGamepad.getLeftY()),
            rx = NebulaConstants.Gamepad.isDriverOneDeadband(driverGamepad.getRightX());

        y = NebulaConstants.Gamepad.cubeInput(y);
        rx = NebulaConstants.Gamepad.cubeInput(rx);
        //TODO:See if this works
        if(Math.abs(drive.getDegreeRoll())> NebulaConstants.Drive.tippingTolerance){
            y = yTipController.calculate(drive.getDegreePitch(), 0);//Make sure this is the right IMU
        }
        drive.arcadeDrive.arcadeDrive(y*multiplier, rx*multiplier);
//        drive.tankDrive(lY * multiplier, rY * multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
