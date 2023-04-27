package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDrive;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;

@Config
@TeleOp(group = "Subsystem test")
public class TeleOpDrivetrainOnly extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;


    // Subsystems
    private MecDrivetrainSubsystem mecDrivetrainSubsystem;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        mecDrivetrainSubsystem = new MecDrivetrainSubsystem(new MecDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        mecDrivetrainSubsystem.init();
        mecDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(mecDrivetrainSubsystem, driverGamepad, false));
    }


    @Override
    public void configureButtons() {
    }

    @Override
    public void matchLoop() {
    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}
