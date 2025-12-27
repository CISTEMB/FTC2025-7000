package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;

@Autonomous(name = "Auto: Lifter Test", group = "Auto")
public class Auto_TestLift extends CommandOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private BeltwaySubsystem beltway;
    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private LauncherMotorsSubsystem launcherMotors;
    private LifterSubsystem lifter;
    private NavigationSubsystem navigation;

    private MecanumVelocityConstraint minVolConstraint = new MecanumVelocityConstraint(25, 25);
    private ProfileAccelerationConstraint minProfAccelConstraint = new ProfileAccelerationConstraint(25);


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        telemetry.update();

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);
        beltway = new BeltwaySubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, telemetry);
        navigation = new NavigationSubsystem(limelight, hardwareMap, AllianceColor.Blue, telemetry);
        launcherMotors = new LauncherMotorsSubsystem(hardwareMap, telemetry, navigation);
        lifter = new LifterSubsystem(hardwareMap, telemetry, navigation);
        lifter.setServoPosition(0.0);

        schedule(
                new SequentialCommandGroup(
                    new InstantCommand(() -> lifter.setServoPosition(0.1), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.2), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.3), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.4), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.5), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.6), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.7), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.8), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.7), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.6), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.5), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.4), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.4), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.3), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.2), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.1), lifter),
                    new WaitCommand(1000),
                    new InstantCommand(() -> lifter.setServoPosition(0.0), lifter)
                )
        );
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
