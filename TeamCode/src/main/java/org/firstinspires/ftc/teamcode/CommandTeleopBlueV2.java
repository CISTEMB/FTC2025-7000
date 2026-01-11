package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.DecreaseLifterPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorStopCommand;
import org.firstinspires.ftc.teamcode.commands.ForwardBeltwayCommand;
import org.firstinspires.ftc.teamcode.commands.HandleLauncherMotorsCommand;
import org.firstinspires.ftc.teamcode.commands.HandleLifterCommand;
import org.firstinspires.ftc.teamcode.commands.IncreaseLifterPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlowRollCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseBeltwayCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SetLifterForPickupCommand;
import org.firstinspires.ftc.teamcode.commands.SetLifterPositionCommand;
import org.firstinspires.ftc.teamcode.commands.StopBeltwayCommand;
import org.firstinspires.ftc.teamcode.commands.StopLauncherMotorsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
//import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@Config
@TeleOp(name = "CommandTeleopV2 Blue", group = "drive")
public class CommandTeleopBlueV2 extends CommandOpMode {
    private MecanumDriveSubsystem drive;
    private LauncherMotorsSubsystem launcherMotors;
    private BeltwaySubsystem beltway;
    private IntakeSubsystem intake;
    private LifterSubsystem lifter;
    private LimelightSubsystem limelight;
    private GamepadEx driverGamepad;
    private NavigationSubsystem navigation;
    private ElevatorMotorsSubsystem elevator; //moving on up in the world like elevators

    private final ElapsedTime runtime = new ElapsedTime();
    private boolean isRed = false;
    private boolean hasStarted = false;
    private Canvas fieldOverlay;
    private TelemetryPacket packet;

    @Override
    public void initialize() {
        // Initialize subsystems
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);

        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStroke("#3F51B5");

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(234)));

        limelight = new LimelightSubsystem(hardwareMap, telemetry);
        navigation = new NavigationSubsystem(limelight, hardwareMap, AllianceColor.Blue, telemetry);
        launcherMotors = new LauncherMotorsSubsystem(hardwareMap, telemetry, navigation);
        beltway = new BeltwaySubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        lifter = new LifterSubsystem(hardwareMap, telemetry, navigation);
        elevator = new ElevatorMotorsSubsystem(hardwareMap, telemetry);

        //default PID adjustments
        launcherMotors.adjustP(100);
        launcherMotors.adjustD(10);
        launcherMotors.adjustF(12);
        launcherMotors.adjustI(.5);
        // Initialize gamepad
        driverGamepad = new GamepadEx(gamepad1);

        // Set up telemetry
        telemetry.setMsTransmissionInterval(11);

        // Register subsystems
        register(drive, launcherMotors, beltway, intake, lifter, limelight);

        // Start limelight
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Lifter Position", lifter.getPosition());
        //telemetry.addData("Instructions", "Press B for Red, X for Blue");
        limelight.limelight.pipelineSwitch(1);

        telemetry.addData("Team", "Blue");
        telemetry.update();
    }

    @Override
    public void run() {
        // Mark that we've started teleop and configure button bindings
        if (!hasStarted && opModeIsActive()) {
            hasStarted = true;
            runtime.reset();
            configureButtonBindings();
            telemetry.addData("Status", "Running");
            telemetry.addData("Motors", "Off");
        }

        launcherMotors.setDefaultCommand(new HandleLauncherMotorsCommand(launcherMotors, navigation));
        lifter.setDefaultCommand(new HandleLifterCommand(lifter, navigation));


        // Run the command scheduler
        super.run();


        // Update limelight
        limelight.read();

        // Update telemetry
        updateTelemetry();
    }

    private void configureButtonBindings() {
        // Set default drive command - matches original arcade drive behavior
        drive.setDefaultCommand(new RunCommand(() -> {
            drive.fastMode = gamepad1.left_trigger >= 0.75;
            drive.arcadeDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
            );
        }, drive));

        // Right trigger >= 0.75: Stop launcher motors
        new Trigger(() -> gamepad1.right_trigger >= 0.75)
            .whenActive(new StopLauncherMotorsCommand(launcherMotors, beltway));

        // Right bumper: Prepare to shoot
//        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//            .whenPressed(new PrepareShootCommandV2(launcherMotors, lifter));


        //right bumper up
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenHeld(new InstantCommand(() -> elevator.goUp()));

        //left bumper down
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new InstantCommand(() -> elevator.goDown()));

        // Y button: Shoot (hold to shoot, release to stop)
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
            .whenHeld(
                    new ParallelCommandGroup(
                            new ReverseBeltwayCommand(beltway),
                            new ReverseIntakeCommand(intake)
                    )
            )
            .whenReleased(
                    new ParallelCommandGroup(
                            new StopBeltwayCommand(beltway),
                            new IntakeStopCommand(intake)
                    )
            );

        //hold Y to move ball to the thrower
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(
                        new ParallelCommandGroup(
                                new ForwardBeltwayCommand(beltway),
                                new IntakeSlowRollCommand(intake)
                        )
                )
                .whenReleased(
                        new ParallelCommandGroup(
                                new StopBeltwayCommand(beltway),
                                new IntakeStopCommand(intake)
                        )
                );

        // X button: Auto-align to target
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(new AutoAlignCommand(drive, navigation, telemetry, isRed));

        // A button: Pickup (hold to run, release to stop)
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(
                new ParallelCommandGroup(
                    new SetLifterForPickupCommand(lifter),
                    new PickupCommand(intake)))
            .whenReleased(new IntakeStopCommand(intake));

        // D-pad up: Increase lifter position
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(
                new SequentialCommandGroup(
                    new IncreaseLifterPositionCommand(lifter)
//                    new PrepareShootCommandV2(launcherMotors, lifter)
                ));

        // D-pad down: Decrease lifter position
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(
                new SequentialCommandGroup(
                    new DecreaseLifterPositionCommand(lifter)
//                    new PrepareShootCommandV2(launcherMotors, lifter)
                ));

        // Back button: Set lifter to pzosition 6
        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
            .whenPressed(new SetLifterPositionCommand(6, lifter));
    }

    private void updateTelemetry() {
        // Periodic updates for all subsystems
        launcherMotors.periodic();
        beltway.periodic();
        lifter.periodic();
        telemetry.update();
    }

    @Override
    public void reset() {
        super.reset();
        limelight.stop();
    }
}
