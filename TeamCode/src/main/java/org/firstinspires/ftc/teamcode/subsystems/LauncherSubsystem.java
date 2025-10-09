package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LauncherSubsystem extends SubsystemBase {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Telemetry t;

    private boolean prepped;

    public LauncherSubsystem (HardwareMap hardwareMap, Telemetry t) {
        this.t = t;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLauncherMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLauncherMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        prepped = false;
    }

    public void prepare_shoot (double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        prepped = true;
    }
    
    public void shoot () {
        if (!prepped) {
            return;
        }
        // TODO: when we add the belt, add support here
    }
    
    public void stop_motors () {
        prepare_shoot(0.0);
        prepped = false;
    }

    public boolean isPrepped () {
        return prepped;
    }
}
