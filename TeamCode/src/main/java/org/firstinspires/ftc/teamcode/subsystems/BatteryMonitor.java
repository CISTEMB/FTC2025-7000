package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Battery Monitoring Subsystem
 * Tracks battery voltage and provides warnings for low battery conditions
 */
public class BatteryMonitor extends SubsystemBase {
    private final VoltageSensor voltageSensor;
    private final Telemetry telemetry;

    // Voltage thresholds
    private static final double VOLTAGE_CRITICAL = 11.5;  // Stop operating
    private static final double VOLTAGE_WARNING = 12.0;   // Warning level
    private static final double VOLTAGE_GOOD = 12.5;      // Good operating level
    private static final double VOLTAGE_FULL = 13.0;      // Fully charged

    // Tracking variables
    private double currentVoltage = 0.0;
    private double minVoltage = Double.MAX_VALUE;
    private double maxVoltage = 0.0;
    private double startVoltage = 0.0;
    private boolean isFirstRead = true;

    // Display options
    private boolean showDetailed = true;

    public BatteryMonitor(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Get the voltage sensor (usually from the control hub)
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        super.periodic();

        // Read current voltage
        currentVoltage = voltageSensor.getVoltage();

        // Track first reading as start voltage
        if (isFirstRead) {
            startVoltage = currentVoltage;
            isFirstRead = false;
        }

        // Update min/max tracking
        if (currentVoltage < minVoltage) {
            minVoltage = currentVoltage;
        }
        if (currentVoltage > maxVoltage) {
            maxVoltage = currentVoltage;
        }

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Display battery information on telemetry
     */
    private void displayTelemetry() {
        telemetry.addData("--- BATTERY STATUS ---", "");
        telemetry.addData("Voltage", "%.2fV %s", currentVoltage, getBatteryIcon());
        telemetry.addData("Status", getBatteryStatus());

        if (showDetailed) {
            telemetry.addData("Battery Level", "%.0f%%", getBatteryPercent());
            telemetry.addData("Voltage Drop", "%.2fV", getVoltageDrop());
            telemetry.addData("Min Voltage", "%.2fV", minVoltage);
            telemetry.addData("Max Voltage", "%.2fV", maxVoltage);
        }

        // Display warning if needed
        if (isCritical()) {
            telemetry.addData("WARNING", "CRITICAL BATTERY - REPLACE NOW!");
        } else if (isLow()) {
            telemetry.addData("WARNING", "Low battery - consider replacing");
        }
    }

    /**
     * Get battery status as a string
     */
    public String getBatteryStatus() {
        if (currentVoltage >= VOLTAGE_FULL) {
            return "Excellent";
        } else if (currentVoltage >= VOLTAGE_GOOD) {
            return "Good";
        } else if (currentVoltage >= VOLTAGE_WARNING) {
            return "Fair";
        } else if (currentVoltage >= VOLTAGE_CRITICAL) {
            return "Low";
        } else {
            return "CRITICAL";
        }
    }

    /**
     * Get a visual battery icon based on voltage level
     */
    public String getBatteryIcon() {
        if (currentVoltage >= VOLTAGE_FULL) {
            return "[████]";
        } else if (currentVoltage >= VOLTAGE_GOOD) {
            return "[███░]";
        } else if (currentVoltage >= VOLTAGE_WARNING) {
            return "[██░░]";
        } else if (currentVoltage >= VOLTAGE_CRITICAL) {
            return "[█░░░]";
        } else {
            return "[░░░░]";
        }
    }

    /**
     * Get estimated battery percentage (rough estimate)
     * Based on typical 12V battery discharge curve
     */
    public double getBatteryPercent() {
        if (currentVoltage >= VOLTAGE_FULL) {
            return 100.0;
        } else if (currentVoltage <= VOLTAGE_CRITICAL) {
            return 0.0;
        } else {
            // Linear approximation between critical and full
            return ((currentVoltage - VOLTAGE_CRITICAL) / (VOLTAGE_FULL - VOLTAGE_CRITICAL)) * 100.0;
        }
    }

    /**
     * Get the voltage drop since start
     */
    public double getVoltageDrop() {
        return startVoltage - currentVoltage;
    }

    /**
     * Check if battery is critically low
     */
    public boolean isCritical() {
        return currentVoltage < VOLTAGE_CRITICAL;
    }

    /**
     * Check if battery is low (but not critical)
     */
    public boolean isLow() {
        return currentVoltage >= VOLTAGE_CRITICAL && currentVoltage < VOLTAGE_WARNING;
    }

    /**
     * Check if battery is in good condition
     */
    public boolean isGood() {
        return currentVoltage >= VOLTAGE_GOOD;
    }

    /**
     * Get current voltage reading
     */
    public double getVoltage() {
        return currentVoltage;
    }

    /**
     * Get minimum voltage recorded
     */
    public double getMinVoltage() {
        return minVoltage;
    }

    /**
     * Get maximum voltage recorded
     */
    public double getMaxVoltage() {
        return maxVoltage;
    }

    /**
     * Get starting voltage
     */
    public double getStartVoltage() {
        return startVoltage;
    }

    /**
     * Toggle detailed telemetry display
     */
    public void setShowDetailed(boolean showDetailed) {
        this.showDetailed = showDetailed;
    }

    /**
     * Reset min/max tracking
     */
    public void resetTracking() {
        minVoltage = currentVoltage;
        maxVoltage = currentVoltage;
        startVoltage = currentVoltage;
    }

    /**
     * Get voltage compensated power multiplier
     * Use this to compensate motor power for voltage drop
     * Example: motor.setPower(power * batteryMonitor.getVoltageCompensation());
     */
    public double getVoltageCompensation() {
        // Compensate based on nominal 12.5V
        double nominalVoltage = 12.5;
        return Math.min(1.5, nominalVoltage / currentVoltage); // Cap at 1.5x
    }
}
