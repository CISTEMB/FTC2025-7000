package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LauncherSubsystem extends SubsystemBase {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private CRServo belt;
    private CRServo pickup1;
    private CRServo pickup2;
    public Servo lifter;
    private AnalogInput lifter_angle;
    private Telemetry t;

    private double motorVelocity = 0.0;

    private boolean prepped;

    public LauncherSubsystem (HardwareMap hardwareMap, Telemetry t) {
        this.t = t;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        belt = hardwareMap.get(CRServo.class, "beltServo");
        pickup1 = hardwareMap.get(CRServo.class, "intakeServo1");
        pickup2 = hardwareMap.get(CRServo.class, "intakeServo2");
        lifter = hardwareMap.get(Servo.class, "lifterServo");
//        lifter_angle = hardwareMap.get(AnalogInput.class, "lifterAngle");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pickup1.setDirection(DcMotorSimple.Direction.REVERSE);
        pickup2.setDirection(DcMotorSimple.Direction.FORWARD);

        prepped = false;
    }

    @Override
    public void periodic() {
        super.periodic();

        t.addData("actual launcher velocity", leftMotor.getVelocity());
        t.addData("targeted launcher velocity", motorVelocity);
        t.addData("belt power", belt.getPower());
        t.addData("prepped", prepped);
    }

    public void prepare_shoot () {
        if (motorVelocity < 950) {
            motorVelocity = 950.0;
        } else if (motorVelocity < 1250) {
            motorVelocity = 1250.0;
        }

        //temp for testing
        leftMotor.setVelocity(motorVelocity);
        rightMotor.setVelocity(motorVelocity);
        prepped = true;
    }

    public void updateMotors(int lifterPos){
        if (lifterPos == 2 || lifterPos == 3){
            motorVelocity = 950;
            leftMotor.setVelocity(motorVelocity);
            rightMotor.setVelocity(motorVelocity);
        } else if (lifterPos == 1){
            motorVelocity = 1200;
            leftMotor.setVelocity(motorVelocity);
            rightMotor.setVelocity(motorVelocity);
        } else {
            leftMotor.setVelocity(0);
            rightMotor.setVelocity(0);
        }

    }


    public void shoot () {
        // belt now goes both ways :O
        belt.setPower(1.0);
        // move intake slowly to help move ball up from second junction to shooting position
        pickup1.setPower(0.1);
        pickup2.setPower(0.1);
    }

    public void shoot_reverse() {
        belt.setPower(-1.0);
    }

    public void stop_shoot () {
        belt.setPower(0.0);
    }

    public void stop_motors () {
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);
        motorVelocity = 0.0;
        belt.setPower(0.0);
        prepped = false;
    }

    public boolean isPrepped () {
        return prepped;
    }

    public void pickup() {
        pickup1.setPower(1);
        pickup2.setPower(1);
    }

    public void pickup_reverse() {
        pickup1.setPower(-0.4); // speed not necessary for reverse
        pickup2.setPower(-0.4);
    }

    public void stop_pickup() {
        pickup1.setPower(0.0);
        pickup2.setPower(0.0);
    }

    public double getLifterAngle() {
        return lifter_angle.getVoltage() / lifter_angle.getMaxVoltage();
    }
}