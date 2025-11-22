package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lifter extends SubsystemBase {

    public Servo servo;
    private Telemetry tm;

    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        tm = telemetry;
        servo = hardwareMap.get(Servo.class, "lifterServo");
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void periodic() {
        super.periodic();
        tm.addData("lifter position", servo.getPosition());
    }
}
