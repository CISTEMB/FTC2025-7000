package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

import javax.annotation.CheckForNull;
import javax.annotation.Nullable;

public class NavigationSubsystem extends SubsystemBase {
    private final MecanumDriveSubsystem drive;
    private final Telemetry telemetry;
    private final LimelightSubsystem limelight;

    private double last_distance;

    @Nullable
    @CheckForNull
    public Pose2d saved_pose;

    //pos x y at 81" -143, -61
    private static final Pose2d blue_apriltag = new Pose2d(-58.34, -55.62, Math.toRadians(-45.0));
    private static final Pose2d red_apriltag = new Pose2d(-58.34, 55.62, Math.toRadians(45.0));
    private final AllianceColor color;

    public NavigationSubsystem(LimelightSubsystem limelightSubsystem, HardwareMap hardwareMap, AllianceColor color, Telemetry telemetry) {
        //TODO: pass this value from the end of the autonomous: https://learnroadrunner.com/advanced.html#transferring-pose-between-opmodes
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);

        TrajectorySequence sequence1 = drive.trajectorySequenceBuilder(new Pose2d(60, -20, Math.toRadians(180))) //starting position
                .back(10)
                .turn(Math.toRadians(30))
                .build();
        sequence1.end();

        drive.setPoseEstimate(sequence1.end());

        this.telemetry = telemetry;
        this.limelight = limelightSubsystem;
        this.color = color;
    }

    public boolean hasSeenTag()
    {
        return saved_pose != null;
    }

    @Override
    public void periodic() {
        //telemetry.addData("Current pose", getPose());
        telemetry.addData("has target", this.hasTarget());

        telemetry.addData("x", drive.getPoseEstimate().getX()); //these values will print out wrong until we scan the april tag
        telemetry.addData("y", drive.getPoseEstimate().getY()); //these values will print out wrong until we scan the april tag
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());

        telemetry.addData("distance from tag", this.getDistance()); //these values will print out wrong until we scan the april tag
        telemetry.addData("angle from tag", this.getAngleOffset()); //these values will print out wrong until we scan the april tag



        telemetry.update();

        if (limelight.result != null && limelight.result.isValid() && limelight.botpose_mt2 != null) {
            Position pos = limelight.botpose_mt2.getPosition().toUnit(DistanceUnit.INCH);


            Position mt1 = limelight.result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);

            telemetry.addData("mt1 x", mt1.x);
            telemetry.addData("mt1 y", mt1.y);
            telemetry.addData("mt2 x", pos.x);
            telemetry.addData("mt2 y", pos.y);

            Pose2d apriltag_pose = this.currentTarget();
            //Pose2d robot_pose = new Pose2d(pos.x - apriltag_pose.getX(), pos.y - apriltag_pose.getY(), limelight.botpose_mt2.getOrientation().getYaw() - apriltag_pose.getHeading());
            Pose2d robot_pose = new Pose2d(mt1.x, mt1.y, limelight.botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES));

            telemetry.addData("robot x", robot_pose.getX());
            telemetry.addData("robot y", robot_pose.getY());

            // Update the drive's pose estimate with limelight position
            drive.setPoseEstimate(robot_pose);

            // Save pose for navigation calculations
            saved_pose = robot_pose;
        }
        drive.update();
    }

    public Pose2d getPose() {
        return drive.getPoseEstimate();
    }

    public Pose2d currentTarget() {
        return color == AllianceColor.Blue ? blue_apriltag : red_apriltag;
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
        double deltaX = this.currentTarget().getX() - currentPose.getX();
        double deltaY = this.currentTarget().getY() - currentPose.getY();
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
        if (limelight != null) {
            return limelight.result != null;
        }
        return false;
    }

    @Nullable
    @CheckForNull
    public Double getDistance() {
        if (saved_pose == null) {
            return last_distance;
        }

        Translation2d target = new Translation2d(this.currentTarget().getX(), this.currentTarget().getY());
        Translation2d robit = new Translation2d(getPose().getX(), getPose().getY());

        double distance = robit.getDistance(target);
        last_distance = distance;
        return distance;
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
