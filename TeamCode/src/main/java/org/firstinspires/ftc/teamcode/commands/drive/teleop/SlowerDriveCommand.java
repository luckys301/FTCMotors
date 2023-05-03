package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDriveSubsystem;

public class SlowerDriveCommand extends DefaultDriveCommand {
    public SlowerDriveCommand(MecDriveSubsystem drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad, false);
        this.multiplier = 0.35;
    }
}