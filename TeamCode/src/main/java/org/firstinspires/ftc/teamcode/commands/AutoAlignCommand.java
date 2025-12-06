package org.firstinspires.ftc.teamcode.commands;

import android.util.Range;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class AutoAlignCommand extends CommandBase {

    private Drive drive;
    private LimelightSubsystem limelight;
    private final Range<Double> alignmentRange = new Range<>(-0.5, 0.5);
    private boolean hasTarget;

    public AutoAlignCommand(Drive drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        hasTarget = limelight.result != null;
    }

    @Override
    public void execute() {
        limelight.read();
        if (limelight.result != null) {
            double x = limelight.result.getTx();
            double turnSpeed = x * 0.05; // <-- turn the robot proportional to tx to have better accuracy

            turnSpeed = Math.max(-0.4, Math.min(0.4, turnSpeed));

            drive.arcadeDrive(0.0, turnSpeed, 0.0, false);
        }
    }

    @Override
    public boolean isFinished() {
        if (limelight.result == null) {
            return true;
        }
        double x = limelight.result.getTx();
        return alignmentRange.contains(x);
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0.0, 0.0, 0.0, false);
    }
}
