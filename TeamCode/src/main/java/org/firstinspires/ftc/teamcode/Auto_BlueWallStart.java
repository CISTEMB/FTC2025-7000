package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomousAutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.HandleLauncherMotorsCommand;
import org.firstinspires.ftc.teamcode.commands.HandleLifterCommand;
import org.firstinspires.ftc.teamcode.commands.SetLauncherSpeedPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SetLifterPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.StopLauncherMotorsCommand;
import org.firstinspires.ftc.teamcode.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto: Blue Wall Start", group = "Auto")
public class Auto_BlueWallStart extends CommandOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private BeltwaySubsystem beltway;
    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private LauncherMotorsSubsystem launcherMotors;
    private LifterSubsystem lifter;
    private LimelightSubsystem limelight;
    private NavigationSubsystem navigation;

    private MecanumVelocityConstraint minVolConstraint = new MecanumVelocityConstraint(25, 25);
    private ProfileAccelerationConstraint minProfAccelConstraint = new ProfileAccelerationConstraint(25);


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Initialized", "true");
        telemetry.update();

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);
        limelight = new LimelightSubsystem(hardwareMap, telemetry);
        navigation = new NavigationSubsystem(limelight, hardwareMap, AllianceColor.Blue, telemetry);
        launcherMotors = new LauncherMotorsSubsystem(hardwareMap, telemetry, navigation);
        beltway = new BeltwaySubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        lifter = new LifterSubsystem(hardwareMap, telemetry, navigation);


        // Register subsystems
        register(drive, launcherMotors, beltway, intake, lifter, limelight);

        // Start limelight
        limelight.start();

        limelight.limelight.pipelineSwitch(1);

        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(new Pose2d(60, -20, Math.toRadians(180))) //starting position
                .back(10, minVolConstraint, minProfAccelConstraint)
                .turn(Math.toRadians(30))
                .build();

        drive.setPoseEstimate(sequence1.start());


//        launcherMotors.leftMotor.setVelocity(750);
//        launcherMotors.rightMotor.setVelocity(750);
//         lifter.setPosition(3);  // Position 3 = 0.8

        TrajectorySequence sequence2 = drive.trajectorySequenceBuilder(sequence1.end())
                .back(5, minVolConstraint, minProfAccelConstraint)
                .strafeRight(25.0)
                .build();

        schedule(
            new SequentialCommandGroup(
                    new TrajectoryFollowerCommand(drive, sequence1),
                    new SetLifterPositionCommand(4.0, lifter),
                    new ParallelCommandGroup(
                        new SetLauncherSpeedPositionCommand(4.0, launcherMotors),
                        new WaitCommand(1600)
                    ),
                    new AutonomousAutoAlignCommand(drive, navigation, false) ,
                    new ShootCommand(beltway, intake, 8250),
                    new StopLauncherMotorsCommand(launcherMotors, beltway),
                    new WaitCommand(1000),
                    new TrajectoryFollowerCommand(drive, sequence2)
            )
        );
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }

    @Override
    public void reset() {
        //stop everything
        launcherMotors.stop();
        beltway.stop();
        intake.stop();
    }
}
