package org.firstinspires.ftc.teamcode.commands;

import android.util.Range;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomousAutoAlignCommand extends CommandBase {

    private MecanumDriveSubsystem drive;
    private NavigationSubsystem navigation;

    private MecanumVelocityConstraint minVolConstraint = new MecanumVelocityConstraint(25, 25);
    private ProfileAccelerationConstraint minProfAccelConstraint = new ProfileAccelerationConstraint(25);

    private Telemetry telemetry;
    private boolean isRed;
    private double offset = 0;

    public AutonomousAutoAlignCommand(MecanumDriveSubsystem drive, NavigationSubsystem navigation, Boolean isRed, Telemetry t) {
        this.telemetry = t;
        this.drive = drive;
        this.navigation = navigation;
        this.isRed = isRed;
        if (this.isRed) {
            offset = 1.5;
        } else {
            offset = -1.5;
        }


        addRequirements(drive);
    }



    @Override
    public void execute() {
        if (navigation.hasSeenTag() && navigation.getAngleOffset() != null) {
            double x = navigation.getAngleOffset();
            double maxAccel = 1.0;
            double targetAngle = x + offset;
            TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0)) //starting position
                    .turn(Math.toRadians(-x), 3.0, 3.0)
                    .build();
            CommandScheduler.getInstance().schedule(
                    new TrajectoryFollowerCommand(drive, sequence1)
            );
        }
    }


    @Override
    public boolean isFinished() {
        if(!navigation.hasTarget()) {
            return true; //if we haven't seen the tag we can't auto aim

        }

//        if (navigation.getAngleOffset() != null) {
//            double x = navigation.getAngleOffset();
//            return navigation.angleMin >= x && x >= navigation.angleMax;
//        }

        return false;
    }


    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0.0, 0.0, 0.0, false);
    }
}
