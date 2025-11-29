package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LauncherMotors extends SubsystemBase {

    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    private Telemetry tm;

    private double motorVelocity = 0.0;
    private boolean prepped = false;

    // PIDF Coefficients for velocity control
    // Start with these values and tune based on your motor performance
    // P: Proportional - how aggressively to correct velocity error
    // I: Integral - corrects steady-state error over time
    // D: Derivative - dampens oscillations
    // F: Feedforward - base power needed to maintain velocity
    private double kP = 10.0;   // Start with 10, increase if response is too slow
    private double kI = 0.5;    // Start small, increase if there's steady-state error
    private double kD = 0.0;    // Usually 0 for velocity control
    private double kF = 12.0;   // Adjust based on motor specs (typically 12-15)

    //825
    private final List<Double> motorVelocities =
            List.of(700.0, 1025.0, 800.0, 800.0, 700.0); // all angles required for normal gameplay

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



        // Apply initial PIDF coefficients
        setPIDFCoefficients(kP, kI, kD, kF);
    }

    public double leftError() {
        return (motorVelocity - leftMotor.getVelocity()) / motorVelocity * 10;
    }


    public double rightError() {
        return (motorVelocity - rightMotor.getVelocity()) / motorVelocity * 10;
    }

    @Override
    public void periodic() {
        super.periodic();
        double actualVelocity = leftMotor.getVelocity();
        double velocityError = motorVelocity - actualVelocity;
        double errorPercent = motorVelocity > 0 ? (velocityError / motorVelocity) * 100 : 0;

        tm.addData("--- LAUNCHER STATUS ---", "");
        tm.addData("Target Velocity", "%.0f ticks/sec", motorVelocity);
        tm.addData("Actual Velocity (L)", "%.0f ticks/sec", actualVelocity);
        tm.addData("Actual Velocity (R)", "%.0f ticks/sec", rightMotor.getVelocity());
        tm.addData("Velocity Error", "%.0f ticks/sec (%.1f%%)", velocityError, errorPercent);
        tm.addData("Prepped", prepped);
        tm.addData("--- PID COEFFICIENTS ---", "");
        tm.addData("P", "%.2f", kP);
        tm.addData("I", "%.2f", kI);
        tm.addData("D", "%.2f", kD);
        tm.addData("F", "%.2f", kF);
    }

    public void prepare(double velocity) {
        motorVelocity = velocity;
        leftMotor.setVelocity(motorVelocity);
        rightMotor.setVelocity(motorVelocity);
        prepped = true;
    }

    public void setSpeedBasedOnLifterPosition(int lifterPosition) {
        double targetVelocity = motorVelocities.get(lifterPosition);
        motorVelocity = targetVelocity;
        leftMotor.setVelocity(targetVelocity);
        rightMotor.setVelocity(targetVelocity);
    }

    public void prepareShoot() {
        if (motorVelocity < 1300) {
            motorVelocity += 25;
        }
        leftMotor.setVelocity(motorVelocity);
        rightMotor.setVelocity(motorVelocity);
        prepped = true;
    }

//    public void stop() {
//        leftMotor.setVelocity(0.0);
//        rightMotor.setVelocity(0.0);
//        motorVelocity = 0.0;
//        prepped = false;
//    }

    public void stop () {
        if (motorVelocity > 0) {
            motorVelocity -= 25;
            leftMotor.setVelocity(motorVelocity);
            rightMotor.setVelocity(motorVelocity);
        }
        if (motorVelocity < 0){
            motorVelocity = 0;
        }
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

    /**
     * Set custom PIDF coefficients for velocity control
     * Call this method to tune the PID controller
     */
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        leftMotor.setVelocityPIDFCoefficients(p, i, d, f);
        rightMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    /**
     * Get the current velocity error (target - actual)
     * Useful for monitoring PID performance
     */
    public double getVelocityError() {
        return motorVelocity - leftMotor.getVelocity();
    }

    /**
     * Get the current velocity error as a percentage
     * Useful for determining if PID is well-tuned
     */
    public double getVelocityErrorPercent() {
        if (motorVelocity == 0) return 0;
        return (getVelocityError() / motorVelocity) * 100.0;
    }

    /**
     * Check if the launcher has reached the target velocity within a tolerance
     * @param tolerancePercent acceptable error percentage (e.g., 5.0 for 5%)
     */
    public boolean isAtTargetVelocity(double tolerancePercent) {
        return Math.abs(getVelocityErrorPercent()) <= tolerancePercent;
    }

    // Individual coefficient adjusters for fine-tuning
    public void adjustP(double delta) {
        setPIDFCoefficients(kP + delta, kI, kD, kF);
    }

    public void adjustI(double delta) {
        setPIDFCoefficients(kP, kI + delta, kD, kF);
    }

    public void adjustD(double delta) {
        setPIDFCoefficients(kP, kI, kD + delta, kF);
    }

    public void adjustF(double delta) {
        setPIDFCoefficients(kP, kI, kD, kF + delta);
    }

    // Getters for current PIDF values
    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }
}
