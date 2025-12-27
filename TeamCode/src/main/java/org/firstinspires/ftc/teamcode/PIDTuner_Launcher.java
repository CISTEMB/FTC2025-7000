package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;
/*
New File: PIDTuner_Launcher.java

  A dedicated tuning OpMode with live adjustment capabilities!

  Gamepad 1 Controls:

  - D-Pad ↑↓ - Adjust target velocity (±50 ticks/sec)
  - Right Trigger - Start launcher
  - Left Trigger - Stop launcher

  Gamepad 2 Controls (PID Tuning):

  - D-Pad ↑↓ - Adjust P (±0.5)
  - D-Pad ←→ - Adjust I (±0.1)
  - Y/A buttons - Adjust F (±0.5)
  - X/B buttons - Adjust D (±0.1)

  How to Tune Your PID:

  1. Run the PIDTuner_Launcher OpMode
  2. Set a target velocity (try 800 ticks/sec to start)
  3. Activate the launcher with right trigger
  4. Watch the telemetry - look at velocity error percentage
  5. Adjust coefficients based on behavior:

    - Too slow to reach target? → Increase P
    - Oscillating/overshooting? → Decrease P
    - Steady-state error (always a bit off)? → Increase I
    - Velocity consistently too low? → Increase F
    - Velocity consistently too high? → Decrease F
  6. Goal: Get velocity error under 5% consistently

  Tips:

  - Start with the default values I provided
  - Make small adjustments and observe the results
  - Tune at the velocity you'll use most often (probably around 800-1000 ticks/sec)
  - Once you find good values, write them down and update the defaults in LauncherMotors.java lines 29-32
  - The tuner shows status messages: EXCELLENT (<2%), GOOD (<5%), OK (<10%), NEEDS TUNING (>10%)

  Your PID tuning system is ready to use! Run the PIDTuner_Launcher OpMode and start experimenting with the
  coefficients.
 */



/**
 * PID Tuning OpMode for Launcher Motors
 *
 * CONTROLS:
 * Gamepad 1:
 *   D-Pad Up/Down    - Increase/Decrease target velocity
 *   Right Trigger    - Activate launcher at target velocity
 *   Left Trigger     - Stop launcher
 *
 * Gamepad 2 (PID Tuning):
 *   D-Pad Up/Down    - Adjust P (±0.5)
 *   D-Pad Left/Right - Adjust I (±0.1)
 *   Y/A              - Adjust F (±0.5)
 *   X/B              - Adjust D (±0.1)
 *
 * HOW TO TUNE:
 * 1. Start with default values (P=10, I=0.5, D=0, F=12)
 * 2. Set a target velocity and activate the launcher
 * 3. Watch the velocity error on telemetry
 * 4. Adjust coefficients:
 *    - If response is too slow: Increase P
 *    - If it oscillates: Decrease P or increase D
 *    - If there's steady-state error: Increase I
 *    - If velocity is consistently low: Increase F
 *    - If velocity is consistently high: Decrease F
 * 5. Goal: Error should be < 5% consistently
 */
@TeleOp(name = "PID Tuner: Launcher", group = "000-Main")
public class PIDTuner_Launcher extends OpMode {
    private LauncherMotorsSubsystem launcher;
    private double targetVelocity = 800.0;
    private final double velocityStep = 50.0;

    // Adjustment amounts
    private final double pAdjust = 0.5;
    private final double iAdjust = 0.1;
    private final double dAdjust = 0.1;
    private final double fAdjust = 0.5;

    // For button debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;

    private boolean lastDpadUp1 = false;
    private boolean lastDpadDown1 = false;

    @Override
    public void init() {
        launcher = new LauncherMotorsSubsystem(hardwareMap, telemetry, new NavigationSubsystem(new LimelightSubsystem(hardwareMap, telemetry), hardwareMap, AllianceColor.Blue, telemetry));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("", "Press RIGHT TRIGGER to start launcher");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Gamepad 1: Control launcher velocity
        if (gamepad1.dpad_up && !lastDpadUp1) {
            targetVelocity += velocityStep;
//            launcher.prepare(targetVelocity);
        }
        if (gamepad1.dpad_down && !lastDpadDown1) {
            targetVelocity -= velocityStep;
            if (targetVelocity < 0) targetVelocity = 0;
//            launcher.prepare(targetVelocity);
        }

        lastDpadUp1 = gamepad1.dpad_up;
        lastDpadDown1 = gamepad1.dpad_down;

        // Activate/Stop launcher
//        if (gamepad1.right_trigger > 0.5) {
//            launcher.prepare(targetVelocity);
//        }
        if (gamepad1.left_trigger > 0.5) {
            launcher.stop();
        }

        // Gamepad 2: Tune PID coefficients
        // Adjust P
        if (gamepad2.dpad_up && !lastDpadUp) {
            launcher.adjustP(pAdjust);
            telemetry.addLine("P INCREASED");
        }
        if (gamepad2.dpad_down && !lastDpadDown) {
            launcher.adjustP(-pAdjust);
            telemetry.addLine("P DECREASED");
        }

        // Adjust I
        if (gamepad2.dpad_right && !lastDpadRight) {
            launcher.adjustI(iAdjust);
            telemetry.addLine("I INCREASED");
        }
        if (gamepad2.dpad_left && !lastDpadLeft) {
            launcher.adjustI(-iAdjust);
            telemetry.addLine("I DECREASED");
        }

        // Adjust F
        if (gamepad2.y && !lastY) {
            launcher.adjustF(fAdjust);
            telemetry.addLine("F INCREASED");
        }
        if (gamepad2.a && !lastA) {
            launcher.adjustF(-fAdjust);
            telemetry.addLine("F DECREASED");
        }

        // Adjust D
        if (gamepad2.x && !lastX) {
            launcher.adjustD(dAdjust);
            telemetry.addLine("D INCREASED");
        }
        if (gamepad2.b && !lastB) {
            launcher.adjustD(-dAdjust);
            telemetry.addLine("D DECREASED");
        }

        // Update button states
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;
        lastY = gamepad2.y;
        lastA = gamepad2.a;
        lastX = gamepad2.x;
        lastB = gamepad2.b;

        // Display tuning status
//        telemetry.addData("=== CONTROLS ===", "");
//        telemetry.addData("GP1 D-Pad ↑↓", "Adjust target velocity");
//        telemetry.addData("GP1 RT", "Start launcher");
//        telemetry.addData("GP1 LT", "Stop launcher");
//        telemetry.addData("", "");
//        telemetry.addData("GP2 D-Pad ↑↓", "Tune P");
//        telemetry.addData("GP2 D-Pad ←→", "Tune I");
//        telemetry.addData("GP2 Y/A", "Tune F");
//        telemetry.addData("GP2 X/B", "Tune D");
//        telemetry.addData("", "");

        // Call periodic to display launcher status
        launcher.periodic();

        // Additional tuning guidance
        double errorPercent = launcher.getVelocityErrorPercent();
        telemetry.addData("=== TUNING STATUS ===", "");
        if (Math.abs(errorPercent) < 2.0) {
            telemetry.addData("Status", "EXCELLENT - Error < 2%");
        } else if (Math.abs(errorPercent) < 5.0) {
            telemetry.addData("Status", "GOOD - Error < 5%");
        } else if (Math.abs(errorPercent) < 10.0) {
            telemetry.addData("Status", "OK - Error < 10%");
        } else {
            telemetry.addData("Status", "NEEDS TUNING - Error > 10%");
        }

        telemetry.update();
    }
}
