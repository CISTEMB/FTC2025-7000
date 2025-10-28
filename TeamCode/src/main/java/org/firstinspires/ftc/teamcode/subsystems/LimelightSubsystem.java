package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Range;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import javax.annotation.Nullable;
import limelight.LimelightHelpers;
public class LimelightSubsystem extends SubsystemBase {
    public static final double margin = 5.0;
    public static final double yaw_to_shoot = 18.0;
    private final Limelight3A limelight;
    private final IMU imu;
    private final IMU.Parameters imuParameters;
    private final Telemetry t;
    public Pose3D botpose_mt2;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry t) {
        this.t = t;

        this.limelight = hardwareMap.get(Limelight3A.class, "limelight-gurtcam");



        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );



        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuParameters);
        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
        double robotYaw = yawPitchRollAngles.getYaw();
        double robotPitch = yawPitchRollAngles.getPitch();
        double robotRoll = yawPitchRollAngles.getRoll();

        // Send the orientation data to the Limelight
        // The last three parameters (tx, ty, ta) are for angular velocity, often set to 0 initially
        LimelightHelpers.SetRobotOrientation("limelight-gurtcam", robotYaw, robotPitch, robotRoll, 0, 0, 0);

        limelight.setPollRateHz(90);

        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void read() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                this.botpose_mt2 = result.getBotpose_MT2();
                t.addData("botpose", botpose_mt2.toString());
            }
        }
    }

    public boolean is_in_shoot_area() {
        return true; // TODO: actually implement this
    }

    public boolean can_shoot() {
        return this.botpose_mt2 != null && Range.create(
                yaw_to_shoot - (margin / 2),
                yaw_to_shoot + (margin / 2)
        ).contains(
                this.botpose_mt2.getOrientation().getYaw()
        ) && this.is_in_shoot_area();
    }
}
