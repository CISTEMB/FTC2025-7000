package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
    private CRServo pickup;
    public CRServo lifter;
    private Telemetry t;

    private Double motorVelocity = 0.0;

    private boolean prepped;

    public LauncherSubsystem (HardwareMap hardwareMap, Telemetry t) {
        this.t = t;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        belt = hardwareMap.get(CRServo.class, "beltServo");
        pickup = hardwareMap.get(CRServo.class, "intakeServo");
        lifter = hardwareMap.get(CRServo.class, "lifterServo");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pickup.setDirection(DcMotorSimple.Direction.FORWARD);

        prepped = false;
    }

    @Override
    public void periodic() {
        super.periodic();

        t.addData("actual launcher velocity", leftMotor.getVelocity());
        t.addData("targetted launcher velocity", motorVelocity);
        t.addData("belt power", belt.getPower());
        t.addData("prepped", prepped);
    }

    public void prepare_shoot () {
        if (motorVelocity < 2500) {
            motorVelocity += 200;
        }

        leftMotor.setVelocity(motorVelocity);
        rightMotor.setVelocity(motorVelocity);
        prepped = true;
    }

    public void shoot () {
        //belt only goes one way
        belt.setPower(1.0);
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
        pickup.setPower(1);
    }

    public void stop_pickup() {
        pickup.setPower(0.0);
    }
}
