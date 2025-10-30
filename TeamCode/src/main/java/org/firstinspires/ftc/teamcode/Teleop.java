package org.firstinspires.ftc.teamcode;

import android.util.Range;

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

            if (limelight.botpose_mt2 != null) {
                telemetry.addData("tx (cm)", limelight.botpose_mt2.getPosition().toUnit(DistanceUnit.CM).x);
            }

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            drive.fastMode = gamepad1.left_trigger >= 0.75;
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

            if (gamepad1.right_trigger >= 0.75 && !launcher.isPrepped()) {
                launcher.prepare_shoot();
            } else if (gamepad1.right_bumper && launcher.isPrepped()) {
                launcher.stop_motors();
            }
            if (gamepad1.y && launcher.isPrepped() && limelight.can_shoot()) {
                launcher.shoot();
            }
            if (gamepad1.x) {
                if (limelight.botpose_mt2 != null) {
                    double x = limelight.botpose_mt2.getPosition().toUnit(DistanceUnit.CM).x;
                    while (limelight.botpose_mt2 != null && !(new Range<>(-5.0, 5.0)).contains(x)) {
                        if (x < 0.0) {
                            drive.arcadeDrive(0.0, -0.2, 0.0, false);
                        } else {
                            drive.arcadeDrive(0.0, 0.2, 0.0, false);
                        }
                        limelight.read();
                        x = limelight.botpose_mt2.getPosition().toUnit(DistanceUnit.CM).x;
                    }

                }
            }
            if (gamepad1.a) {
                launcher.pickup();
            } else {
                launcher.stop_pickup();
            }

            launcher.periodic();
            telemetry.update();
        }

        limelight.stop();
    }
}
