package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.List;

@TeleOp(name = "Teleop", group = "000-Main")
public class Teleop extends LinearOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private IMU.Parameters imuParameters;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, telemetry);
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(11);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        limelight.start();

        telemetry.addData("Status", "Running");
        telemetry.addData("Motors", "Off");
        telemetry.update();

        while (opModeIsActive()) {
            limelight.read();

            drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

            if (launcher.isPrepped()) {
                telemetry.addData("Motors", "On");
            } else {
                telemetry.addData("Motors", "Off");
            }

            if (limelight.can_shoot()) {
                telemetry.addData("Can shoot", "Yes");
            } else {
                telemetry.addData("Can shoot", "No");
            }

            if (gamepad1.x) {
                launcher.prepare_shoot();
            } else if (gamepad1.b && launcher.isPrepped()) {
                launcher.stop_motors();
            }
            if (gamepad1.y && launcher.isPrepped() && limelight.can_shoot()) {
                launcher.shoot();
            }

            launcher.periodic();
            telemetry.update();
        }

        limelight.stop();
    }
}
