package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Deprecated
@Disabled
@Autonomous(name="Autonomous: RightSideScoreSpecimen", group="000Real")
public class OLD_Autonomous_RightSideScoreSpecimen extends CommandOpMode {

    private ElevatorV2 elevatorV2;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private SampleMecanumDrive drive;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private Climber climber;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        elevatorV2 = new ElevatorV2(hardwareMap, telemetry);
        worm = new Worm(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry, true);
        wrist.SetSpeed(0.5);
        climber = new Climber(hardwareMap, telemetry);
        climber.Goto(0);

        drive = new SampleMecanumDrive(hardwareMap);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(drive, false);

        //place the robot just to the left of the right tape parking area
        Trajectory rightTapeToScoringPosition = mecanumDriveSubsystem.trajectoryBuilder(new Pose2d(26.16, -62.72, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(5.06, -62.44), new AngularVelocityConstraint(12), new ProfileAccelerationConstraint(12))
                .build();

        drive.setPoseEstimate(rightTapeToScoringPosition.start());

        SequentialCommandGroup scoreSpecimenGroup = new SequentialCommandGroup(
                new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 23.00),
                new WormSetPowerCommand(worm, 0.2).interruptOn(() -> worm.getAngle() > 27.0), //final target: 28.5
                new ParallelCommandGroup(
                    new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() >= 11.0),
                    new WristSetAngleCommand(wrist, 132.0) //132.0
                ),
                new ElevatorV2ExtendCommand(elevatorV2, 0.5).interruptOn(() -> elevatorV2.getDistanceInInches() >= 12.8)
        );

        SequentialCommandGroup resetElevatorGroup = new SequentialCommandGroup(
                new ElevatorV2RetractCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.isRetracted()),
                new WormSetPowerCommand(worm, -1).interruptOn(() -> worm.getAngle() <= -5)
        );

        Trajectory pushForward = mecanumDriveSubsystem.trajectoryBuilder(rightTapeToScoringPosition.end())
                .lineToConstantHeading(new Vector2d(5.06, -37.69), new AngularVelocityConstraint(6), new ProfileAccelerationConstraint(6))
                .build();

        Trajectory backup = mecanumDriveSubsystem.trajectoryBuilder(pushForward.end())
                .lineToConstantHeading(new Vector2d(5.06, -47.69))
                .build();

        Trajectory parkOnRight = mecanumDriveSubsystem.trajectoryBuilder(backup.end())
                .splineTo(new Vector2d(39.66, -25.03), Math.toRadians(90.00))
                .splineTo(new Vector2d(25.31, -11.67), Math.toRadians(180.00))
                .build();

        schedule(
                new SequentialCommandGroup(
                        new TrajectoryFollowerCommand(mecanumDriveSubsystem, rightTapeToScoringPosition),
                        scoreSpecimenGroup,
                        new TrajectoryFollowerCommand(mecanumDriveSubsystem, pushForward),
                        new ParallelDeadlineGroup(
                                new TrajectoryFollowerCommand(mecanumDriveSubsystem, backup),
                                new GrabberPickupCommand(grabber)
                        ),
                        resetElevatorGroup,
                        new TrajectoryFollowerCommand(mecanumDriveSubsystem, parkOnRight)
                )
        );

//                scoreSpecimenGroup,
//                new TrajectoryFollowerCommand(mecanumDriveSubsystem, pushForward));

    }

    @Override
    public void run() {
        super.run();
        elevatorV2.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go
        worm.SetElevatorDistanceInInches(elevatorV2.getHorizontalExtension());
        telemetry.update();
    }
}
