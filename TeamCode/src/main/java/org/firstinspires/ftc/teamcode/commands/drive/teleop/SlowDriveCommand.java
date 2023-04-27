package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;

public class SlowDriveCommand extends DefaultDriveCommand {
    public SlowDriveCommand(MecDrivetrainSubsystem drive, GamepadEx driverGamepad, boolean isFieldCentric) {
        super(drive, driverGamepad, isFieldCentric);
        this.multiplier = 0.3;
    }
}