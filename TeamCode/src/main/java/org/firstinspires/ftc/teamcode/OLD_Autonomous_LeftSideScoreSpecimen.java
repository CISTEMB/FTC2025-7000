//package org.firstinspires.ftc.teamcode;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
//import org.firstinspires.ftc.teamcode.commands.roadrunner.TurnCommand;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Climber;
//import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
//import org.firstinspires.ftc.teamcode.subsystems.Grabber;
//import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Worm;
//import org.firstinspires.ftc.teamcode.subsystems.Wrist;
//
//@Deprecated
//@Disabled
//@Autonomous(name="Autonomous: LeftSideScoreSpecimen", group="000Real")
//public class OLD_Autonomous_LeftSideScoreSpecimen extends CommandOpMode {
//
//    private ElevatorV2 elevatorV2;
//    private Worm worm;
//    private Grabber grabber;
//    private Wrist wrist;
//    private SampleMecanumDrive drive;
//    private MecanumDriveSubsystem mecanumDriveSubsystem;
//    private Climber climber;
//
//    @Override
//    public void initialize() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.addData("intialized", "true");
//        elevatorV2 = new ElevatorV2(hardwareMap, telemetry);
//        worm = new Worm(hardwareMap, telemetry);
//        grabber = new Grabber(hardwareMap, telemetry);
//        wrist = new Wrist(hardwareMap, telemetry, true);
//        wrist.SetSpeed(0.5);
//        climber = new Climber(hardwareMap, telemetry);
//        climber.Goto(0);
//
//        telemetry.addData("worm angle", worm.getAngle());
//        telemetry.addData("elevator distance inches", elevatorV2.getDistanceInInches());
//        telemetry.addData("elevator motor distance", elevatorV2.getDistance());
//        telemetry.addData("wrist angle", wrist.getAngle());
//        telemetry.update();
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        mecanumDriveSubsystem = new MecanumDriveSubsystem(drive, false);
//
//        //place the robot just to the left of the right tape parking area
//        Trajectory leftTapeToScoringPosition = mecanumDriveSubsystem.trajectoryBuilder(new Pose2d(-38.76, -63.58, Math.toRadians(90.00)))
//                .lineToConstantHeading(new Vector2d(-11.64, -61.58))
//                .build();
//
//        drive.setPoseEstimate(leftTapeToScoringPosition.start());
//
//        SequentialCommandGroup scoreSpecimenGroup = new SequentialCommandGroup(
//                new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 23.00),
//                new WormSetPowerCommand(worm, 0.2).interruptOn(() -> worm.getAngle() > 27.0), //final target: 28.5
//                new ParallelCommandGroup(
//                    new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() >= 11.0),
//                    new WristSetAngleCommand(wrist, 132.0) //132.0
//                ),
//                new ElevatorV2ExtendCommand(elevatorV2, 0.5).interruptOn(() -> elevatorV2.getDistanceInInches() >= 12.5)
//        );
//
//        SequentialCommandGroup resetElevatorGroup = new SequentialCommandGroup(
//                new ElevatorV2RetractCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() <= 2),
//                new ElevatorV2RetractCommand(elevatorV2, 0.2).interruptOn(() -> elevatorV2.getDistanceInInches() <= 2),
//                new WormSetPowerCommand(worm, -0.5).interruptOn(() -> worm.getAngle() <= -3.5)
//        );
//
//        SequentialCommandGroup resetElevatorGroupTwo = new SequentialCommandGroup(
//                new ElevatorV2RetractCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() <= 2),
//                new ElevatorV2RetractCommand(elevatorV2, 0.2).interruptOn(() -> elevatorV2.getDistanceInInches() <= 2),
//                new WormSetPowerCommand(worm, -0.5).interruptOn(() -> worm.getAngle() <= -4.5)
//        );
//
//        Trajectory pushForward = mecanumDriveSubsystem.trajectoryBuilder(leftTapeToScoringPosition.end())
//                .lineToConstantHeading(new Vector2d(-12.64, -37.69), new AngularVelocityConstraint(12), new ProfileAccelerationConstraint(12))
//                .build();
//
//        Trajectory backup = mecanumDriveSubsystem.trajectoryBuilder(pushForward.end())
//                .lineToConstantHeading(new Vector2d(-13.64, -47.69))
//                .build();
//
//        Trajectory driveToFirstSample = mecanumDriveSubsystem.trajectoryBuilder(backup.end())
//                .lineToConstantHeading(new Vector2d(-52.00, -47.69), new AngularVelocityConstraint(24), new ProfileAccelerationConstraint(24))
//                .build();
//
//        Trajectory pickupToFirstSample = mecanumDriveSubsystem.trajectoryBuilder(driveToFirstSample.end())
//                .lineToConstantHeading(new Vector2d(-52.50, -38.00), new AngularVelocityConstraint(4), new ProfileAccelerationConstraint(4))
//                .build();
//
//        Trajectory dontBonkSecondSamples = mecanumDriveSubsystem.trajectoryBuilder(pickupToFirstSample.end())
//                .lineToConstantHeading(new Vector2d(-52.50, -42.00), new AngularVelocityConstraint(24), new ProfileAccelerationConstraint(24))
//                .build();
//
//
////        Trajectory turnToFaceBasket = mecanumDriveSubsystem.trajectoryBuilder(dontBonkSecondSamples.end())
////                .splineTo(new Vector2d(-54.68, -55.38), Math.toRadians(225.00), new AngularVelocityConstraint(4), new ProfileAccelerationConstraint(4))
////                .build();
//
//        Trajectory moveToDunk = mecanumDriveSubsystem.trajectoryBuilder(new Pose2d(-52.50, -42.00, Math.toRadians(225.00)))
//                .lineToConstantHeading(new Vector2d(-60.08, -60.22), new AngularVelocityConstraint(14), new ProfileAccelerationConstraint(14))
//                .build();
//
//        Trajectory victoryScoot = mecanumDriveSubsystem.trajectoryBuilder(moveToDunk.end())
//                .lineToConstantHeading(new Vector2d(-54.68, -55.38))
//                .build();
//
//        Trajectory leftPark = mecanumDriveSubsystem.trajectoryBuilder(new Pose2d(-54.96, -55.80, Math.toRadians(90.00)))
//                .splineTo(new Vector2d(-42.11, -25.49), Math.toRadians(90.00))
//                .splineTo(new Vector2d(-24.79, -12.22), Math.toRadians(0.00))
//                .build();
//
//
//
////        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d(-60.13, -61.66, Math.toRadians(90.00)))
////                .splineTo(new Vector2d(-42.53, -32.05), Math.toRadians(29.49))
////                .splineTo(new Vector2d(-24.51, -10.68), Math.toRadians(-2.23))
////                .build();
//
//
//        schedule(
//            new SequentialCommandGroup(
//                    new ParallelCommandGroup(
//                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, leftTapeToScoringPosition),
//                    scoreSpecimenGroup
//                ),
//                new TrajectoryFollowerCommand(mecanumDriveSubsystem, pushForward),
//                new ParallelDeadlineGroup(
//                        new WaitCommand(500),
//                        new GrabberDropCommand(grabber)
//                ),
//                new ParallelDeadlineGroup(
//                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, backup)
//                ),
//                new ParallelCommandGroup(
//                    resetElevatorGroup,
//                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, driveToFirstSample)
//                ),
//                new ParallelCommandGroup(
//                    new WristSetAngleCommand(wrist, 150),
//                    new ElevatorV2ExtendCommand(elevatorV2, 0.8).interruptOn(() -> elevatorV2.getDistanceInInches() >= 6)
//                ),
//                new ParallelCommandGroup(
//                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, pickupToFirstSample),
//                    new ParallelRaceGroup(
//                        new GrabberPickupCommand(grabber),
//                        new WaitCommand(2400)
//                    )
//                ),
//                new ParallelCommandGroup(
//                    new ElevatorV2RetractCommand(elevatorV2, 1.0).interruptOn(() -> elevatorV2.isRetracted()),
//                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, dontBonkSecondSamples)
//                ),
//                new ParallelCommandGroup(
//                    new TurnCommand(mecanumDriveSubsystem, Math.toRadians(135)),
//                    new SequentialCommandGroup(
//                        new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 65),
//                        new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.isVerticallyExtended())
//                    )
//                ),
//                new TrajectoryFollowerCommand(mecanumDriveSubsystem, moveToDunk),
//                new ParallelRaceGroup(
//                    new GrabberPickupCommand(grabber),
//                    new WaitCommand(1750)
//                ),
//                new TrajectoryFollowerCommand(mecanumDriveSubsystem, victoryScoot),
//                new TurnCommand(mecanumDriveSubsystem, Math.toRadians(-135)),
//                new ParallelCommandGroup(
//                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, leftPark),
//                    resetElevatorGroupTwo
//                )
//            )
//        );
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        elevatorV2.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go
//        worm.SetElevatorDistanceInInches(elevatorV2.getHorizontalExtension());
//        telemetry.addData("worm angle", worm.getAngle());
//        telemetry.addData("elevator distance inches", elevatorV2.getDistanceInInches());
//        telemetry.addData("elevator motor distance", elevatorV2.getDistance());
//        telemetry.addData("wrist angle", wrist.getAngle());
//        telemetry.update();
//    }
//}
