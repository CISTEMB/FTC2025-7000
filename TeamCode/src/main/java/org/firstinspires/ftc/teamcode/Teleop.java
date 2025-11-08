package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private Boolean stalling = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, telemetry);
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(11);

        boolean isRed = true;

        while (opModeInInit()) {
            if (gamepad1.b) {
                limelight.limelight.pipelineSwitch(0);
                isRed = true;
            } else if (gamepad1.x) {
                limelight.limelight.pipelineSwitch(1);
                isRed = false;
            }
            if (isRed) {
                telemetry.addData("Team", "Red");
                telemetry.update();
            } else {
                telemetry.addData("Team", "Blue");
                telemetry.update();
            }
        }

        runtime.reset();

        limelight.start();

        telemetry.addData("Status", "Running");
        telemetry.addData("Motors", "Off");
        telemetry.update();

        while (opModeIsActive()) {
            limelight.read();
            telemetry.addData("IsPrepped", launcher.isPrepped());
            telemetry.addData("CanShoot", limelight.can_shoot());

            LLResult result = limelight.result;
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

            if (gamepad1.right_trigger >= 0.75) {
                launcher.stop_motors();
            }

            if(gamepad1.rightBumperWasPressed() && gamepad1.rightBumperWasReleased()) {
                launcher.prepare_shoot();
            }

//            if (gamepad1.right_bumper) {
//                launcher.prepare_shoot();
//            }

            if (gamepad1.y) {
                launcher.shoot();
            } else {
                launcher.stop_shoot();
            }
            if (gamepad1.x) {
                if (limelight.result != null) {
                    double x = limelight.result.getTx();
                    while (limelight.result != null && !(new Range<>(-5.0, 5.0)).contains(x)) {
                        if (x < 0.0) {
                            drive.arcadeDrive(0.0, -0.5, 0.0, false);
                        } else {
                            drive.arcadeDrive(0.0, 0.5, 0.0, false);
                        }
                        limelight.read();
                        x = limelight.result.getTx();
                    }
                }
            }
            if (gamepad1.a) {
                launcher.pickup();
            } else {
                launcher.stop_pickup();
            }

            if (gamepad1.dpad_down) {
                launcher.lifter.setDirection(DcMotorSimple.Direction.FORWARD);
                launcher.lifter.setPower(0.25);
                stalling = true;
            } else if (gamepad1.dpad_up) {
                launcher.lifter.setDirection(DcMotorSimple.Direction.REVERSE);
                launcher.lifter.setPower(0.25);
                stalling = true;
            } else if (stalling) {
                launcher.lifter.setDirection(DcMotorSimple.Direction.REVERSE);
                launcher.lifter.setPower(0.067);
            } else {
                launcher.lifter.setPower(0.0);
            }

            launcher.periodic();
            telemetry.update();
        }

        stalling = false;
        launcher.lifter.setPower(0.0);
        limelight.stop();
    }
}
