package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Lifter extends SubsystemBase {

    public Servo servo;
    private AnalogInput angleInput;
    private Telemetry tm;
    private final int increment = 1;
    private final int minPosition = 0;
    private final int maxPosition = 4;
    private int currentPosition = 0;

    private final List<Double> lifterPosList =
            List.of(0.0, 0.3, 0.6, 0.8, 1.0, 0.8); // all angles required for normal gameplay

    public void increasePosition() {
        if (currentPosition < maxPosition) {
            currentPosition += increment;
            servo.setPosition(lifterPosList.get(currentPosition));
        }
    }


    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        tm = telemetry;
        servo = hardwareMap.get(Servo.class, "lifterServo");
    }

    public void setServoPosition(double position) {
        servo.setPosition(position);
        tm.addData("setServoPosition", position);
    }

    public void setPosition(int position) {
        if (position <= lifterPosList.size()) {
            this.currentPosition = position;
            servo.setPosition(lifterPosList.get(currentPosition));
        }
    }

    public int getPosition() {
        return currentPosition;
    }



    public void decreasePosition() {
        if (currentPosition > minPosition) {
            currentPosition -= increment;
            servo.setPosition(lifterPosList.get(currentPosition));
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
        tm.addData("lifter target position", lifterPosList.get(currentPosition));
        tm.addData("lifter actual position", servo.getPosition());
    }
}
