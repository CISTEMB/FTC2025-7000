//package org.firstinspires.ftc.teamcode;
//
//import android.util.Range;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.button.Trigger;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
//import org.firstinspires.ftc.teamcode.subsystems.Drive;
//import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
//
//@TeleOp(name = "Teleop V2", group = "000-Main")
//public class CommandTeleop extends CommandOpMode {
//    private LimelightSubsystem limelight;
//    private LauncherSubsystem launcher;
//    private IMU imu;
//    private IMU.Parameters imuParameters;
//    private final ElapsedTime runtime = new ElapsedTime();
//    private Drive drive;
//    private GamepadEx gamepadEx;
//
//    public void configV1() {
//        new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >  0.5).whileActiveContinuous(
//            new InstantCommand(() -> drive.fastMode = true)
//        ).whenInactive(
//            new InstantCommand(() -> drive.fastMode = false)
//        );
//
//        new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).toggleWhenActive(
//            new InstantCommand(() -> launcher.stop_motors())
//        );
//
//        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//            new InstantCommand(() -> launcher.prepare_shoot())
//        );
//
//        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//            new InstantCommand(() -> {
//                if (launcher.isPrepped() && limelight.can_shoot()){
//                    launcher.shoot();
//                }
//            }
//            )
//        );
//
//            if (gamepad1.y && launcher.isPrepped() && limelight.can_shoot()) {
//                launcher.shoot();
//            } else {
//                launcher.stop_shoot();
//            }
//            if (gamepad1.x) {
//                if (limelight.result != null) {
//                    double x = limelight.result.getTx();
//                    while (limelight.result != null && !(new Range<>(-5.0, 5.0)).contains(x)) {
//                        if (x < 0.0) {
//                            drive.arcadeDrive(0.0, -0.5, 0.0, false);
//                        } else {
//                            drive.arcadeDrive(0.0, 0.5, 0.0, false);
//                        }
//                        limelight.read();
//                        x = limelight.result.getTx();
//                    }
//                }
//            }
//            if (gamepad1.a) {
//                launcher.pickup();
//            } else {
//                launcher.stop_pickup();
//            }
//
//            if (gamepad1.dpad_down) {
//                launcher.lifter.setDirection(DcMotorSimple.Direction.REVERSE);
//                launcher.lifter.setPower(0.25);
//            } else if (gamepad1.dpad_up) {
//                launcher.lifter.setDirection(DcMotorSimple.Direction.FORWARD);
//                launcher.lifter.setPower(0.25);
//            } else {
//                launcher.lifter.setPower(0.03);
//            }
//
//        drive.setDefaultCommand(
//                new DriveWithGamepadCommand(gamepad1, drive)
//        );
//    }
//
//    @Override
//    public void initialize() {
//        runtime.reset();
//        gamepadEx = new GamepadEx(gamepad1);
//        drive = new Drive(hardwareMap, telemetry);
//        launcher = new LauncherSubsystem(hardwareMap, telemetry);
//        limelight = new LimelightSubsystem(hardwareMap, telemetry, new LEDSubsystem(hardwareMap, telemetry));
//        telemetry.addData("Status", "Initialized");
//        telemetry.setMsTransmissionInterval(11);
//
//        limelight.start();
//
//        boolean isRed = true;
//
//        if (gamepad1.b) {
//            limelight.limelight.pipelineSwitch(0);
//            isRed = true;
//        } else if (gamepad1.x) {
//            limelight.limelight.pipelineSwitch(1);
//            isRed = false;
//        }
//        if (isRed) {
//            telemetry.addData("Team", "Red");
//            telemetry.update();
//        } else {
//            telemetry.addData("Team", "Blue");
//            telemetry.update();
//        }
//
//        configV1();
//    }
//
//    @Override
//    public void reset() {
//        super.reset();
//        launcher.lifter.setPower(0.0);
//        limelight.stop();
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        limelight.read();
//
//        telemetry.addData("Status", "Running");
//        telemetry.addData("Motors", "Off");
//        telemetry.addData("IsPrepped", launcher.isPrepped());
//        telemetry.addData("CanShoot", limelight.can_shoot());
//
//        if (limelight.can_shoot()) {
//            telemetry.addData("Can shoot", "Yes");
//        } else {
//            telemetry.addData("Can shoot", "No");
//        }
//
//        if (launcher.isPrepped()) {
//            telemetry.addData("Motors", "On");
//        } else {
//            telemetry.addData("Motors", "Off");
//        }
//
//        LLResult result = limelight.result;
//        if (result != null && result.isValid()) {
//            double tx = result.getTx(); // How far left or right the target is (degrees)
//            double ty = result.getTy(); // How far up or down the target is (degrees)
//            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
//
//            telemetry.addData("Target X", tx);
//            telemetry.addData("Target Y", ty);
//            telemetry.addData("Target Area", ta);
//        } else {
//            telemetry.addData("Limelight", "No Targets");
//        }
//
//        launcher.periodic();
//        telemetry.update();
//    }
//}
