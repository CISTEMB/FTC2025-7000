package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp(name = "Teleop", group = "000-Main")
public class Teleop extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap, telemetry);
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.addData("Motors", "Off");
        telemetry.update();

        while (opModeIsActive()) {
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
            if (gamepad1.y) {
                launcher.shoot();
            }

            telemetry.update();
        }
    }
}
