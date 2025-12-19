package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

import javax.annotation.CheckForNull;
import javax.annotation.Nullable;

public class NavigationSubsystem extends SubsystemBase {
    private final StandardTrackingWheelLocalizer localizer;
    private final Telemetry telemetry;
    private final LimelightSubsystem limelight;

    @Nullable
    @CheckForNull
    public Pose2d saved_pose;

    public NavigationSubsystem(LimelightSubsystem limelightSubsystem, HardwareMap hardwareMap, Telemetry telemetry) {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        this.telemetry = telemetry;
        this.limelight = limelightSubsystem;
    }

    @Override
    public void periodic() {
        localizer.update();
        telemetry.addData("Current pose", getPose());

        if (limelight.result != null && limelight.result.isValid() && limelight.botpose_mt2 != null) {
            Position pos = limelight.botpose_mt2.getPosition().toUnit(DistanceUnit.INCH);

            // Create pose from limelight data
            Pose2d limelightPose = new Pose2d(pos.x, pos.y, limelight.botpose_mt2.getOrientation().getYaw());

            // Update the drive's pose estimate with limelight position
            localizer.setPoseEstimate(limelightPose);

            // Save pose for navigation calculations
            saved_pose = limelightPose;
        }
    }

    public Pose2d getPose() {
        return localizer.getPoseEstimate();
    }

    @Nullable
    @CheckForNull
    public Double getAngleOffset() {
        if (limelight.result != null) {
            return limelight.result.getTx();
        } else if (saved_pose == null) {
            return null;
        }

        Pose2d currentPose = getPose();

        // Calculate the angle from current position to saved_pose
        double deltaX = saved_pose.getX() - currentPose.getX();
        double deltaY = saved_pose.getY() - currentPose.getY();
        double angleToTarget = Math.atan2(deltaY, deltaX);

        // Calculate the difference between current heading and angle to target
        double angleOffset = angleToTarget - currentPose.getHeading();

        // Normalize to [-PI, PI]
        while (angleOffset > Math.PI) angleOffset -= 2 * Math.PI;
        while (angleOffset < -Math.PI) angleOffset += 2 * Math.PI;

        // Convert to degrees to match limelight's getTx() format
        return Math.toDegrees(angleOffset);
    }

    public boolean hasTarget() {
        return limelight.result != null;
    }

    @Nullable
    @CheckForNull
    public Double getDistance() {
        if (saved_pose == null) {
            return null;
        }
        Pose2d pose = getPose();
        return Math.sqrt(Math.pow(pose.getX() - saved_pose.getX(), 2) + Math.pow(pose.getY() - saved_pose.getY(), 2));
    }

    //              closer   --- further
    // lift angle   1.0      ---  0.7
    // motor speed  700.0    ---  885.0
    // inches dist  24       ---  177.0 in
    @Nullable
    @CheckForNull
    public Double getPosition() {
        Double distance = getDistance();
        if (distance == null) {
            return null;
        }
        return Math.min(Math.max((distance / 177) * 4, 0.0), 4.0);
    }
}
