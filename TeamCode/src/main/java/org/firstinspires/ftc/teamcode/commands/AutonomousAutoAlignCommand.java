package org.firstinspires.ftc.teamcode.commands;

import android.util.Range;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;

public class AutonomousAutoAlignCommand extends CommandBase {

    private MecanumDriveSubsystem drive;
    private NavigationSubsystem navigation;

    private boolean isRed;
    private double turnSpeed = 0.0;

    public AutonomousAutoAlignCommand(MecanumDriveSubsystem drive, NavigationSubsystem navigation, Boolean isRed) {

        this.drive = drive;
        this.navigation = navigation;
        this.isRed = isRed;

        addRequirements(drive);
    }


    @Override
    public void execute() {
        if (navigation.hasSeenTag() && navigation.getAngleOffset() != null) {

            double x = navigation.getAngleOffset();
            turnSpeed = Math.pow(x, 3) * 0.0067 + 0.1 * -Math.signum(x);   // <-- turn the robot proportional to tx to have better accuracy
            // Note: x^2 * 0.067 + 0.04 while maintaining whether x is positive or negative

            turnSpeed = Math.max(-0.4, Math.min(0.4, turnSpeed));
            if (Double.isNaN(turnSpeed)) turnSpeed = 0;

            drive.arcadeDrive(0.0, turnSpeed, 0.0, false);

        }
    }


    @Override
    public boolean isFinished() {
        if(!navigation.hasTarget()) {
            return true; //if we haven't seen the tag we can't auto aim

        }

        if (navigation.getAngleOffset() != null) {
            double x = navigation.getAngleOffset();
            return navigation.angleMin >= x && x >= navigation.angleMax;
        }

        return false;
    }


    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0.0, 0.0, 0.0, false);
    }
}
