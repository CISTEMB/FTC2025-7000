package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OLD Teleop", group = "000-Main")
@Disabled
public class Teleop extends LinearOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private IMU.Parameters imuParameters;
    private final ElapsedTime runtime = new ElapsedTime();
    private Integer lifterPosition;
    private final List<Double> lifterPosList =
            List.of(0.0, 0.3, 0.6, 0.8, 1.0); // all angles required for normal gameplay
    // final private Double lifterIncrement = 0.1;
    private Pose2d saved_pose;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, telemetry);
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(11);

        boolean isRed = true;
        lifterPosition = 0;
        if (launcher.lifter.getPosition() > 0.9){
            lifterPosition = 4;
        }

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
            localizer.update();
            telemetry.addData("IsPrepped", launcher.isPrepped());
            telemetry.addData("CanShoot", limelight.can_shoot());
            telemetry.addData("Lifter Position", lifterPosition);

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
            drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

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
                launcher.slow_motors();
            }

            if(gamepad1.rightBumperWasPressed() && gamepad1.rightBumperWasReleased()) {
                launcher.prepare_shoot();
            }

//            if (gamepad1.right_bumper) {
//                launcher.prepare_shoot();
//            }

            if (gamepad1.y) {
                launcher.shoot();
            } else if (!gamepad1.b){
                launcher.stop_shoot();
            }

            if(gamepad1.b){
                launcher.shoot_reverse();
                launcher.pickup_reverse();
            }

            if (saved_pose != null) {
                telemetry.addData("Saved pose", saved_pose);
                Pose2d pose = localizer.getPoseEstimate();
                telemetry.addData("Current pose", pose);
                double a = pose.getX() - saved_pose.getX();
                double b = pose.getY() - saved_pose.getY();
                double c = Math.atan(b / a);
                double x = localizer.getPoseEstimate().getHeading() - c;
                telemetry.addData("a", a);
                telemetry.addData("b", b);
                telemetry.addData("c", c);
                telemetry.addData("x", x);
            }

            if (gamepad1.x) {
                if (saved_pose != null) {
                    Pose2d pose = localizer.getPoseEstimate();
                    double a = pose.getX() - saved_pose.getX();
                    double b = pose.getY() - saved_pose.getY();
                    double c = Math.atan(b / a);
                    double x = localizer.getPoseEstimate().getHeading() - c;
                    while (!(new Range<>(Math.toRadians(-5.0), Math.toRadians(5.0))).contains(x) && opModeIsActive()) {
                        if (x < 0.0) {
                            drive.arcadeDrive(0.0, 0.3, 0.0, false);
                        } else {
                            drive.arcadeDrive(0.0, -0.3, 0.0, false);
                        }
                        localizer.update();
                        x = localizer.getPoseEstimate().getHeading() - c;
                        pose = localizer.getPoseEstimate();
                        telemetry.addData("Saved pose", saved_pose);
                        telemetry.addData("Current pose", pose);
                        telemetry.addData("a", a);
                        telemetry.addData("b", b);
                        telemetry.addData("c", c);
                        telemetry.addData("x", x);
                        telemetry.update();
                    }

                } else if (limelight.result != null) {
                    double x = limelight.result.getTx();
                    while (limelight.result != null && !(new Range<>(-5.0, 5.0)).contains(x) && opModeIsActive()) {
                        if (x < 0.0) {
                            drive.arcadeDrive(0.0, -0.3, 0.0, false);
                        } else {
                            drive.arcadeDrive(0.0, 0.3, 0.0, false);
                        }
                        limelight.read();
                        x = limelight.result.getTx();
                        localizer.update();
                    }
                    assert limelight.result != null;
                    Position pos = limelight.result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH);
                    saved_pose = localizer.getPoseEstimate();
                    saved_pose = new Pose2d(saved_pose.getX() + pos.x, saved_pose.getY() + pos.y);
                }
            }
            if (gamepad1.a) {
                launcher.pickup();
            } else if (!gamepad1.b && !gamepad1.y) {
                launcher.stop_pickup();

            }

            if (gamepad1.dpadDownWasPressed() && gamepad1.dpadDownWasReleased()) {
                if (lifterPosition > 0) {
                    lifterPosition -= 1;
                    launcher.lifter.setPosition(lifterPosList.get(lifterPosition));
                    launcher.updateMotors(lifterPosition);
                }
                // stalling = true;
                // stalling shouldn't be required for non-continuous
            } else if (gamepad1.dpadUpWasPressed() && gamepad1.dpadUpWasReleased()) {
                if (lifterPosition < 4) {
                    lifterPosition += 1;
                    launcher.lifter.setPosition(lifterPosList.get(lifterPosition));
                    launcher.updateMotors(lifterPosition);
                }
                // stalling = true;
            } /* else if (stalling) {
                launcher.lifter.setDirection(DcMotorSimple.Direction.REVERSE);
                launcher.lifter.setPower(0.07);
            } else {
                launcher.lifter.setPower(0.0);
            } */


            launcher.periodic();
            telemetry.update();
        }

        limelight.stop();
    }
}