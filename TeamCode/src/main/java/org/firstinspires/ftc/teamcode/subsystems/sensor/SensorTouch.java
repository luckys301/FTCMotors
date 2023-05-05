package org.firstinspires.ftc.teamcode.subsystems.sensor;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SensorTouch extends SubsystemBase implements HardwareDevice {
    private final Telemetry telemetry;
    private final TouchSensor touchSensor;
    public SensorTouch(HardwareMap hardwareMap , Telemetry tl) {
        this.telemetry = tl;
        this.touchSensor = hardwareMap.get(TouchSensor.class, "digitalTouch");
    }

    public void periodic() {
        telemetry.update();
    }

    public boolean isPressed(){
        return touchSensor.isPressed();
        //true if the touch sensor is being presse
        //false if not being pressed
    }

    @Override
    public void disable() {touchSensor.close();}

    @Override
    public String getDeviceType() {return "Color Sensor";}

}