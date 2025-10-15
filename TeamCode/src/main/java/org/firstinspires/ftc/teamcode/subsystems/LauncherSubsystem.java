package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LauncherSubsystem extends SubsystemBase {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private CRServo belt;
    private Telemetry t;

    private boolean prepped;

    public LauncherSubsystem (HardwareMap hardwareMap, Telemetry t) {
        this.t = t;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");
        belt = hardwareMap.get(CRServo.class, "beltServo");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        prepped = false;
    }

    @Override
    public void periodic() {
        super.periodic();

        t.addData("launcher power", leftMotor.getPower());
    }

    public void prepare_shoot () {
        double power = leftMotor.getPower();
        if (power < 1.0) {
            power += 1;
        } else {
            power = 0.0;
        }
        t.addData("launcher power", power);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        prepped = true;
    }
    
    public void shoot () throws InterruptedException {
        if (!prepped) {
            return;
        }
        belt.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setPower(1.0);
        Thread.sleep(500);
        belt.setPower(0.0);
    }
    
    public void stop_motors () {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        belt.setPower(0.0);
        prepped = false;
    }

    public boolean isPrepped () {
        return prepped;
    }
}
