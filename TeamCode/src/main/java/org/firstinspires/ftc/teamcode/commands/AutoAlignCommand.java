package org.firstinspires.ftc.teamcode.commands;

import android.util.Range;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class AutoAlignCommand extends CommandBase {

    private Drive drive;
    private LimelightSubsystem limelight;
    private Telemetry telemetry;
    private Range<Double> alignmentRange;
    private boolean isRed;
    private double turnSpeed = 0.0;
    private boolean hasTarget;

    public AutoAlignCommand(Drive drive, LimelightSubsystem limelight, Telemetry telemetry, Boolean isRed) {
        this.drive = drive;
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.isRed = isRed;

        if (this.isRed) {
            alignmentRange = new Range<>(-1.5, -0.5);
        } else {
            alignmentRange = new Range<>(0.5, 1.5);
        }

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        hasTarget = limelight.result != null;
    }

    @Override
    public void execute() {
        telemetry.addData("turn speed", turnSpeed);

        limelight.read();
        if (limelight.result != null) {
            double x = limelight.result.getTx();
            turnSpeed = x * Math.abs(x) * 0.0067 + 0.1 * Math.signum(x);   // <-- turn the robot proportional to tx to have better accuracy
            // Note: x^2 * 0.067 + 0.04 while maintaining whether x is positive or negative

            //turnSpeed = Math.max(-0.4, Math.min(0.4, turnSpeed));

            if (Double.isNaN(turnSpeed)) turnSpeed = 0;
            drive.arcadeDrive(0.0, turnSpeed, 0.0, false);
        }
    }

    @Override
    public boolean isFinished() {

        if (limelight.result == null) {
            return true;
        }

        double x = limelight.result.getTx();
        telemetry.addData("angle distance", x);

        return alignmentRange.contains(x);
    }


    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0.0, 0.0, 0.0, false);
    }
}
