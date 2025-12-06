package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.IncreaseLifterPositionCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.PrepareShootCommandV2;
import org.firstinspires.ftc.teamcode.commands.SetLifterPosition;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.StopLauncherMotorsCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Beltway;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotors;
import org.firstinspires.ftc.teamcode.subsystems.Lifter;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto: Red Goal Start", group = "Auto")
public class Auto_RedGoalStart extends CommandOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Beltway beltway;
    private MecanumDriveSubsystem drive;
    private Intake intake;
    private LauncherMotors launcherMotors;
    private Lifter lifter;

    private MecanumVelocityConstraint minVolConstraint = new MecanumVelocityConstraint(12, 18);
    private ProfileAccelerationConstraint minProfAccelConstraint = new ProfileAccelerationConstraint(12);


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);

        beltway = new Beltway(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        launcherMotors = new LauncherMotors(hardwareMap, telemetry);
        lifter = new Lifter(hardwareMap, telemetry);


        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(new Pose2d(-49.5, 49.5, Math.toRadians(126))) //starting position
                .back(16, minVolConstraint, minProfAccelConstraint)
                .turn(Math.toRadians(5))
                .build();

        drive.setPoseEstimate(sequence1.start());

//        launcherMotors.leftMotor.setVelocity(750);
//        launcherMotors.rightMotor.setVelocity(750);
//         lifter.setPosition(3);  // Position 3 = 0.8

        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(sequence1.end())
                .setReversed(true)
                .splineTo(new Vector2d(-12, 28), Math.toRadians(90))
                .setReversed(false)
                .build();

        // activate intake while driving forward, make sure to activate top belt slightly (0.25s) about half way through to move up top ball

        TrajectorySequence sequence3 = drive.trajectorySequenceBuilder(sequence2.end())
                .back(21, new MecanumVelocityConstraint(12, 18), new ProfileAccelerationConstraint(12))
                .build();

        TrajectorySequence sequence4 = drive.trajectorySequenceBuilder(sequence3.end())
                .splineToLinearHeading(new Pose2d(-40.5, 36.5, Math.toRadians(126)), Math.toRadians(135))
                .build();

        // shoot again, angle around 0.7 this time

        TrajectorySequence sequence5 = drive.trajectorySequenceBuilder(sequence4.end())
                .strafeLeft(12)
//                .waitSeconds(3)
//
//                        .turn(Math.toRadians(90))
//                        .forward(30)
//                        .turn(Math.toRadians(90))
//                        .forward(30)
//                        .turn(Math.toRadians(90))
//                        .forward(30)
//                        .turn(Math.toRadians(90))
                .build();

        schedule(
                new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(drive, sequence1),
                        new WaitCommand(1000),
                        //new SetLifterPosition(0.8, lifter),
                        //new PrepareShootCommandV2(launcherMotors, lifter),
                        //new ShootCommand(beltway, intake),
                        //new StopLauncherMotorsCommand(launcherMotors, beltway),
                        new WaitCommand(1000),
                        new TrajectoryFollowerCommand(drive, sequence2),
                        new ParallelCommandGroup(
                                new TrajectoryFollowerCommand(drive, sequence3)
                                //new PickupCommand(intake)
                        ),
                        new TrajectoryFollowerCommand(drive, sequence4),
                        new TrajectoryFollowerCommand(drive, sequence3),
                        //new SetLifterPosition(0.7, lifter),
                        //new PrepareShootCommandV2(launcherMotors, lifter),
                        //new ShootCommand(beltway, intake),
                        //new StopLauncherMotorsCommand(launcherMotors, beltway),
                        new TrajectoryFollowerCommand(drive, sequence5)
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }
}
