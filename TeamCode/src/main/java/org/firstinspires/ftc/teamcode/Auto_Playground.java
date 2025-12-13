package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.List;

@Autonomous(name = "Auto: Playground", group = "Auto", preselectTeleOp = "Teleop")
public class Auto_Playground extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap, telemetry);
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap, telemetry);
//        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);

        // Start the Limelight
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Info", "Detecting AprilTag for position...");
        telemetry.update();

        // Detect AprilTag to determine starting position
        String startPosition = "Unknown";
        int detectedTagId = -1;
        Pose3D robotPose = null;

        // Try to detect AprilTag for up to 3 seconds
        ElapsedTime detectionTimer = new ElapsedTime();
        detectionTimer.reset();

        while (!isStarted() && !isStopRequested()) {
            limelight.read();

            if (limelight.result != null && limelight.botpose_mt2 != null) {
                // Get the detected AprilTag information
                List<LLResultTypes.FiducialResult> fiducials = limelight.result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    // Get the first (closest/most confident) detected tag
                    LLResultTypes.FiducialResult detectedTag = fiducials.get(0);
                    detectedTagId = (int) detectedTag.getFiducialId();
                    robotPose = limelight.botpose_mt2;

                    // Determine starting position based on AprilTag ID
                    startPosition = getStartPositionFromTag(detectedTagId);

                    telemetry.addData("Status", "AprilTag Detected!");
                    telemetry.addData("Tag ID", detectedTagId);
                    telemetry.addData("Starting Position", startPosition);
                    telemetry.addData("Robot X", "%.2f", robotPose.getPosition().x);
                    telemetry.addData("Robot Y", "%.2f", robotPose.getPosition().y);
                    telemetry.addData("Robot Z", "%.2f", robotPose.getPosition().z);
                } else {
                    telemetry.addData("Status", "Searching for AprilTag...");
                }
            } else {
                telemetry.addData("Status", "Searching for AprilTag...");
                telemetry.addData("Detection Time", "%.1f sec", detectionTimer.seconds());
            }

            telemetry.update();
            sleep(50);
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.addData("Starting Position", startPosition);
        telemetry.addData("Detected Tag", detectedTagId);
        telemetry.update();

        // Your autonomous code here - now you know the starting position!
        drive.runToPosition(20, 0.8);

        sleep(15000);

        limelight.stop();
    }

    /**
     * Map AprilTag IDs to starting positions on the field
     * Customize these based on your field setup and game manual
     */
    private String getStartPositionFromTag(int tagId) {
        switch (tagId) {
            case 1:
                return "Red Alliance - Left";
            case 2:
                return "Red Alliance - Center";
            case 3:
                return "Red Alliance - Right";
            case 4:
                return "Blue Alliance - Left";
            case 5:
                return "Blue Alliance - Center";
            case 6:
                return "Blue Alliance - Right";
            // Add more cases based on your field configuration
            default:
                return "Unknown Position (Tag " + tagId + ")";
        }
    }
}
