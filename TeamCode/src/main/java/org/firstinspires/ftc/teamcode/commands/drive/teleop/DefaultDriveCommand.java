package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.mecDrive.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

public class DefaultDriveCommand extends CommandBase {
    private MecDriveSubsystem drive;
    private GamepadEx driverGamepad;

    protected double multiplier;
    boolean isFieldCentric;

    public static PIDCoefficients X_TIPING_PID = new PIDCoefficients(3, 0, 0);
    public static PIDController xTipController = new PIDController(X_TIPING_PID.kP, X_TIPING_PID.kI, X_TIPING_PID.kD);
    public static PIDCoefficients Y_TIPING_PID = new PIDCoefficients(3, 0, 0);
    public static PIDController yTipController = new PIDController(X_TIPING_PID.kP, X_TIPING_PID.kI, X_TIPING_PID.kD);

    public DefaultDriveCommand(MecDriveSubsystem drive, GamepadEx driverGamepad, boolean isFieldCentric) {
        this.drive = drive;
        this.driverGamepad = driverGamepad;
        this.multiplier = 1.0;
        addRequirements(this.drive);

        this.isFieldCentric = isFieldCentric;
        xTipController.setTolerance(NebulaConstants.Drive.tippingTolerance);
    }

    @Override
    public void execute() {
//        if(driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {// ToDO:IS the other command working or is this:
//            multiplier = 0.3;
//        } else {
//            multiplier = 1;
//        }
        double y = squareInput(driverGamepad.getLeftY()),
            x = squareInput(driverGamepad.getLeftX()),
            rx = squareInput(driverGamepad.getRightX());

        //TODO:See if this works
        if(Math.abs(drive.getDegreeRoll())> NebulaConstants.Drive.tippingTolerance){
            x= xTipController.calculate(drive.getDegreePitch(), 0);
        }
        if(Math.abs(drive.getDegreePitch())> NebulaConstants.Drive.tippingTolerance){
            y= yTipController.calculate(drive.getDegreePitch(), 0);
        }
        drive.fieldCentric(
            (y * multiplier),
            (x * multiplier),
            -(rx * multiplier)
        );

    }



    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    public static double squareInput(double value) {
        return value * Math.abs(value);
    }

}
