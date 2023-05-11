package org.firstinspires.ftc.teamcode.subsystems.shooterHood;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class shooterHoodServo extends SubsystemBase
{
    public enum ShooterPos {
        IN(0.51),
        OUT (0.5),
        ;

        public final double shooterPos;
        ShooterPos(double shooterPos) {
            this.shooterPos = shooterPos;
        }
    }
    Telemetry telemetry;
    private final ServoEx clawS1;     //Claw

    public shooterHoodServo(Telemetry tl, HardwareMap hw) {
        this.clawS1 = new SimpleServo(hw, "clawS2", 0, 360);
        this.clawS1.setPosition(ShooterPos.IN.shooterPos);  //Port 3

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Servo 1 Pos: ", clawS1.getPosition());
    }

    public void setPosition(double pos) {
        clawS1.setPosition(pos);
    }
    public Command setPositionCommand(shooterHoodServo.ShooterPos pos) {
        return new InstantCommand(()->{setPosition(pos.shooterPos);});
    }
}