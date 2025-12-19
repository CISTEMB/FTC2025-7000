package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BeltwaySubsystem extends SubsystemBase {

    public CRServo servo;

    private Telemetry tm;

    public BeltwaySubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        servo = hardwareMap.get(CRServo.class, "beltServo");
        tm = telemetry;
    }

    public void forward() {
        servo.setPower(1);
    }

    public void stop() {
        servo.setPower(0.0);
    }

    public void reverse() {
        servo.setPower(-1);
    }

    @Override
    public void periodic() {
        super.periodic();
        tm.addData("belt power", servo.getPower());
    }
}