package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherMotors extends SubsystemBase {

    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    private Telemetry tm;

    private double motorVelocity = 0.0;
    private boolean prepped = false;

    public LauncherMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        tm = telemetry;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        super.periodic();
        tm.addData("actual launcher velocity", leftMotor.getVelocity());
        tm.addData("targeted launcher velocity", motorVelocity);
        tm.addData("prepped", prepped);
    }

    public void prepare(double velocity) {
        motorVelocity = velocity;
        leftMotor.setVelocity(motorVelocity);
        rightMotor.setVelocity(motorVelocity);
        prepped = true;
    }

    public void prepareDefault() {
        if (motorVelocity < 950) {
            motorVelocity = 950.0;
        } else if (motorVelocity < 1250) {
            motorVelocity = 1250.0;
        }
        prepare(motorVelocity);
    }

    public void stop() {
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);
        motorVelocity = 0.0;
        prepped = false;
    }

    public boolean isPrepped() {
        return prepped;
    }

    public double getVelocity() {
        return motorVelocity;
    }

    public double getActualVelocity() {
        return leftMotor.getVelocity();
    }
}
