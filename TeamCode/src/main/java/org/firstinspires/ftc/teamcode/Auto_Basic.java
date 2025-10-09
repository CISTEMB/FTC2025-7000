package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous(name = "Auto: Basic Drive", group = "Auto", preselectTeleOp = "Test_Launcher")
public class Auto_Basic extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap, telemetry);
//        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.update();

        drive.runToPosition(20, 0.8);

        sleep(15000);
    }
}
