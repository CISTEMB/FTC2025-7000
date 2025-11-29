package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {

    public CRServo servo1;
    public CRServo servo2;
    private Telemetry tm;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        tm = telemetry;
        servo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        servo2 = hardwareMap.get(CRServo.class, "intakeServo2");

        servo1.setDirection(DcMotorSimple.Direction.REVERSE);
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void forward() {
        servo1.setPower(1);
        servo2.setPower(1);
    }

    public void stop() {
        servo1.setPower(0);
        servo2.setPower(0);
    }

    public void reverse() {
        servo1.setPower(-0.4);
        servo2.setPower(-0.4);
    }

    public void slowRoll() {
        servo1.setPower(0.4);
        servo2.setPower(0.4);
    }
}