package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class ElevatorMotorsSubsystem extends SubsystemBase {

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    private final Telemetry tm;

    public ElevatorMotorsSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        tm = telemetry;

        leftMotor = hardwareMap.get(DcMotor.class, "leftElevatorMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightElevatorMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void periodic() {
        super.periodic();
    }


    public void stop () {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void goUp() {
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
    }

    public void goDown() {
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
    }
}
