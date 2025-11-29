package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BatteryMonitor;

/**
 * Test OpMode for Battery Monitor
 *
 * This demonstrates how to use the BatteryMonitor subsystem
 * and displays battery information on the driver station
 *
 * CONTROLS:
 *   A - Toggle detailed view
 *   X - Reset min/max tracking
 */
@TeleOp(name = "Battery Monitor Test", group = "Testing")
public class BatteryMonitorTest extends OpMode {
    private BatteryMonitor batteryMonitor;
    private boolean lastA = false;
    private boolean lastX = false;

    @Override
    public void init() {
        batteryMonitor = new BatteryMonitor(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Battery monitor ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle detailed view with A button
        if (gamepad1.a && !lastA) {
            batteryMonitor.setShowDetailed(true);
        }
        lastA = gamepad1.a;

        // Reset tracking with X button
        if (gamepad1.x && !lastX) {
            batteryMonitor.resetTracking();
            telemetry.addLine("Tracking reset!");
        }
        lastX = gamepad1.x;

        // Call periodic to update and display battery info
        batteryMonitor.periodic();

        // Add control hints
        telemetry.addData("", "");
        telemetry.addData("Controls:", "A = Detailed view | X = Reset");

        // Example of using voltage compensation
        double voltageComp = batteryMonitor.getVoltageCompensation();
        telemetry.addData("", "");
        telemetry.addData("--- VOLTAGE COMPENSATION ---", "");
        telemetry.addData("Multiplier", "%.3f", voltageComp);
        telemetry.addData("Example", "Use this to maintain consistent motor power");

        telemetry.update();
    }
}
