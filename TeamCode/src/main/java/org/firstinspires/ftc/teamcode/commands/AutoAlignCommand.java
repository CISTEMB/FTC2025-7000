package org.firstinspires.ftc.teamcode.commands;

import android.util.Range;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Navigation;

public class AutoAlignCommand extends CommandBase {

    private Drive drive;
    private Navigation navigation;
    private Telemetry telemetry;
    private final Range<Double> alignmentRange = new Range<>(-0.5, 0.5);
    private boolean isRed;

    private double angleOffset;
    private double turnSpeed = 0.0;
    private boolean hasTarget;

    public AutoAlignCommand(Drive drive, Navigation navigation, Telemetry telemetry, Boolean isRed) {
        this.drive = drive;
        this.navigation = navigation;
        this.telemetry = telemetry;
        this.isRed = isRed;

        if (this.isRed) {
            angleOffset = 0.5;
        } else {
            angleOffset = -0.5;
        }

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        hasTarget = navigation.hasTarget();
    }

    @Override
    public void execute() {
        telemetry.addData("turn speed", turnSpeed);


        if (navigation.hasTarget() && navigation.getAngleOffset() != null) {
            double x = navigation.getAngleOffset() + angleOffset;
            turnSpeed = x * Math.abs(x) * 0.0067 + 0.1 * Math.signum(x);   // <-- turn the robot proportional to tx to have better accuracy
            // Note: x^2 * 0.067 + 0.04 while maintaining whether x is positive or negative

            turnSpeed = Math.max(-0.4, Math.min(0.4, turnSpeed));

            if (Double.isNaN(turnSpeed)) turnSpeed = 0;
            drive.arcadeDrive(0.0, turnSpeed, 0.0, false);
        }
    }

    @Override
    public boolean isFinished() {

        if (!navigation.hasTarget()) {
            return true;
        }

        if (navigation.getAngleOffset() != null) {
            double x = navigation.getAngleOffset() + angleOffset;
            telemetry.addData("angle distance", x);
            return alignmentRange.contains(x);
        }

        return false;
    }


    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0.0, 0.0, 0.0, false);
    }
}
