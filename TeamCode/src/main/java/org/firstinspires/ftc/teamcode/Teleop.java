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

import java.util.List;

@TeleOp(name = "Teleop", group = "000-Main")
public class Teleop extends LinearOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private IMU.Parameters imuParameters;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap, telemetry);
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(11);

        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        limelight = hardwareMap.get(Limelight3A.class, "limelight-gurtcam");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuParameters);

        //limelight.setPollRateHz(90);

        limelight.pipelineSwitch(0);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        limelight.start();

        telemetry.addData("Status", "Running");
        telemetry.addData("Motors", "Off");
        telemetry.update();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

//            double Yaw   = orientation.getYaw(AngleUnit.DEGREES);
//            double Pitch = orientation.getPitch(AngleUnit.DEGREES);
//            double Roll  = orientation.getRoll(AngleUnit.DEGREES);
//            telemetry.addData("imuYaw", Yaw);
//            telemetry.addData("imuPitch", Pitch);
//            telemetry.addData("imuRoll", Roll);

            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    }

                    //                    // Access general information
//                    Pose3D botpose = result.getBotpose();
//                    double captureLatency = result.getCaptureLatency();
//                    double targetingLatency = result.getTargetingLatency();
//                    double parseLatency = result.getParseLatency();
//                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                    telemetry.addData("Parse Latency", parseLatency);
//                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("txnc", result.getTxNC());
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("tync", result.getTyNC());
//
//                    telemetry.addData("Botpose", botpose.toString());
//
//                    // Access barcode results
//                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
//                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                        telemetry.addData("Barcode", "Data: %s", br.getData());
//                    }
//
//                    // Access classifier results
//                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
//                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
//                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
//                    }
//
//                    // Access detector results
//                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//                    for (LLResultTypes.DetectorResult dr : detectorResults) {
//                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
//                    }
//
//                    // Access fiducial results
//                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                    }
//
//                    // Access color results
//                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                    for (LLResultTypes.ColorResult cr : colorResults) {
//                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//                    }
                }
            }

            drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

            if (launcher.isPrepped()) {
                telemetry.addData("Motors", "On");
            } else {
                telemetry.addData("Motors", "Off");
            }

            if (gamepad1.x && !launcher.isPrepped()) {
                launcher.prepare_shoot(1.0);
            } else if (gamepad1.b && launcher.isPrepped()) {
                launcher.stop_motors();
            }
            if (gamepad1.y && launcher.isPrepped()) {
                launcher.shoot();
            }
            if (gamepad1.a) {

            }

            telemetry.update();
        }

        limelight.stop();
    }
}
