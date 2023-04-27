package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

public class SlowerDriveCommand extends DefaultDriveCommand {
    public SlowerDriveCommand(MecDrivetrainSubsystem drive, GamepadEx driverGamepad) {
        super(drive, driverGamepad, false);
        this.multiplier = 0.35;
    }
}