package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Optional;

import javax.annotation.CheckForNull;
import javax.annotation.Nullable;

public class Navigation extends SubsystemBase {
    private final StandardTrackingWheelLocalizer localizer;
    private final Telemetry telemetry;
    private final LimelightSubsystem limelight;
    @Nullable
    @CheckForNull
    public Pose2d saved_pose;
    public Navigation(LimelightSubsystem limelightSubsystem, HardwareMap hardwareMap, Telemetry telemetry) {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        this.telemetry = telemetry;
        this.limelight = limelightSubsystem;
    }

    @Override
    public void periodic() {
        localizer.update();
        telemetry.addData("Current pose", getPose());

        if (limelight.result != null && limelight.result.isValid()) {
            Position pos = limelight.botpose_mt2.getPosition().toUnit(DistanceUnit.INCH);
            saved_pose = getPose();
            saved_pose = new Pose2d(saved_pose.getX() + pos.x, saved_pose.getY() + pos.y);
        }
    }

    public Pose2d getPose() {
        return localizer.getPoseEstimate();
    }

    @Nullable
    @CheckForNull
    public Double getAngleOffset() {
        if (saved_pose != null) {
            if (limelight.result != null) {
                return limelight.result.getTx();
            }
        }

        return null;
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
