package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp(name = "Teleop", group = "000-Main")
public class Teleop extends LinearOpMode {
    private Limelight3A limelight;
    private IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap, telemetry);
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight-gurtcam");
        imu = hardwareMap.get(IMU.class, "imu");

        limelight.setPollRateHz(90);

        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        limelight.start();

        telemetry.addData("Status", "Running");
        telemetry.addData("Motors", "Off");
        telemetry.update();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Bot pose", botpose.toString());
                    telemetry.addData("Bot orientation", botpose.getOrientation());
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
    }
}
