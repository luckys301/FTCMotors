package org.firstinspires.ftc.teamcode.commands.drive.teleop.tank;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.sixWheelDrive.TankDriveSubsystem;

public class DefaultTankDriveCommand extends CommandBase {
    private final TankDriveSubsystem drive;
    private final GamepadEx driverGamepad;

    protected double multiplier;

    public DefaultTankDriveCommand(TankDriveSubsystem drive, GamepadEx driverGamepad) {
        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
//        drive.arcadeDrive.arcadeDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightX() * multiplier);
        drive.tankDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightY() * multiplier);
        // Mecanum - driverGamepad.getLeftY() * multiplier, driverGamepad.getLeftX() * multiplier, driverGamepad.getRightX() * multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
