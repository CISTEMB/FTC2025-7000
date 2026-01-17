package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.lib.commands.DeferredCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Set;

public class AutonomousAutoAlignCommandRyan extends SequentialCommandGroup {

    private MecanumDriveSubsystem drive;
    private NavigationSubsystem navigation;

    private MecanumVelocityConstraint minVolConstraint = new MecanumVelocityConstraint(25, 25);
    private ProfileAccelerationConstraint minProfAccelConstraint = new ProfileAccelerationConstraint(25);

    private Telemetry telemetry;
    private boolean isRed;
    private Double offset = 0.0;
    private Double colorBump = 0.0;
    private TrajectorySequence sequence1;

    public AutonomousAutoAlignCommandRyan(MecanumDriveSubsystem drive, NavigationSubsystem navigation, Boolean isRed, Telemetry t) {
        this.telemetry = t;
        this.drive = drive;
        this.navigation = navigation;
        this.isRed = isRed;
        if (this.isRed) {
            colorBump = 1.5;
        } else {
            colorBump = -1.5;
        }

        addCommands(
                new WaitUntilCommand(() -> {
                    telemetry.addData("Auto Align state", "waiting");
                     offset = navigation.getAngleOffset();

                    return navigation.hasSeenTag() && offset != null;
                }),
                new InstantCommand(() -> {
                    telemetry.addData("Auto Align state", "Building trajectory");
                    double x = offset;
                    double maxAccel = 1.0;
                    double targetAngle = offset + colorBump;
                    telemetry.addData("Auto Align Turn offset", x);
                    sequence1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate()) //starting position
//                             .forward(12)
                            .turn(Math.toRadians(-x), 3.0, 3.0)
                            .build();

                    telemetry.addData("Auto Align Sequence start", sequence1.start().toString());
                    telemetry.addData("Auto Align Sequence End", sequence1.end().toString());

                }),
                new ParallelDeadlineGroup(
                    new DeferredCommand(() -> new TrajectoryFollowerCommand(drive, sequence1), Set.of(drive)),
                    new RunCommand(() -> {
                        telemetry.addData("Auto Align state", "Running trajectory");
                        telemetry.addData("Auto Align Sequence start (follower)", sequence1.start().toString());
                        telemetry.addData("Auto Align Sequence End (follower)", sequence1.end().toString());

                    })
                ),
                new InstantCommand(() -> telemetry.addData("Auto Align state", "Done"))
        );

    }


//
//    @Override
//    public void execute() {
//        if (navigation.hasSeenTag() && navigation.getAngleOffset() != null) {
//            double x = navigation.getAngleOffset();
//            double maxAccel = 1.0;
//            double targetAngle = x + offset;
//            TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0)) //starting position
//                    .turn(Math.toRadians(-x), 3.0, 3.0)
//                    .build();
//            CommandScheduler.getInstance().schedule(
//                    new TrajectoryFollowerCommand(drive, sequence1)
//            );
//        }
//    }
//
//
//    @Override
//    public boolean isFinished() {
//        if(!navigation.hasTarget()) {
//            return true; //if we haven't seen the tag we can't auto aim
//
//        }
//
////        if (navigation.getAngleOffset() != null) {
////            double x = navigation.getAngleOffset();
////            return navigation.angleMin >= x && x >= navigation.angleMax;
////        }
//
//        return false;
//    }
//
//
//    @Override
//    public void end(boolean interrupted) {
//        drive.arcadeDrive(0.0, 0.0, 0.0, false);
//    }
}
