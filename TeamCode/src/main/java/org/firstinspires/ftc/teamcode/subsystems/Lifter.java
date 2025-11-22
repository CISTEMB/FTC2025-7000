package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lifter extends SubsystemBase {

    public Servo servo;
    private AnalogInput angleInput;
    private Telemetry tm;
    private double position;
    private final double increment = 0.1;
    private final double minPosition = 0.0;
    private final double maxPosition = 0.1;

    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        tm = telemetry;
        servo = hardwareMap.get(Servo.class, "lifterServo");
        // angleInput = hardwareMap.get(AnalogInput.class, "lifterAngle");
        position = servo.getPosition();
    }

    public void setPosition(double position) {
        this.position = position;
        servo.setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    public void increasePosition() {
        if (position < maxPosition) {
            position += increment;
            servo.setPosition(position);
        }
    }

    public void decreasePosition() {
        if (position > minPosition) {
            position -= increment;
            servo.setPosition(position);
        }
    }

    public double getAngle() {
        // TODO: Uncomment when angle sensor is connected
        // return angleInput.getVoltage() / angleInput.getMaxVoltage();
        return 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        tm.addData("lifter position", position);
    }
}
