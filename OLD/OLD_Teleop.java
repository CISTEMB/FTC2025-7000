package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Deprecated
@Disabled
@TeleOp(name="TeleOp", group="000Real")
public class OLD_Teleop extends CommandOpMode {

    private Elevator elevator;
    private ElevatorV2 elevatorV2;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private Drive drive;
    private GamepadEx driver;
    private Climber climber;

    private int elevatorDistance = 0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
//        elevator = new Elevator(hardwareMap, telemetry);
        elevatorV2 = new ElevatorV2(hardwareMap, telemetry);
        elevatorDistance = elevatorV2.getDistance();
        worm = new Worm(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry, false);
        wrist.SetSpeed(0.5);
        climber = new Climber(hardwareMap, telemetry);
        climber.Goto(0);

        drive = new Drive(hardwareMap, telemetry);

        driver = new GamepadEx(gamepad1);

        //configV2(); //current competition config
        testingConfigV3();

    }

    public void configV1() {
        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new ElevatorExtendCommand(elevator)) ;
        driver.getGamepadButton(GamepadKeys.Button.B).whileHeld(new ElevatorRetractCommand(elevator));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new WormLowerCommand(worm,1));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new WormRaiseCommand(worm, 1));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new GrabberPickupCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new GrabberDropCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new RunCommand(wrist::addFifteen, wrist));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new RunCommand(wrist::subFifteen, wrist));
    }

    public void configV2() {
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new ElevatorExtendCommand(elevator));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new ElevatorRetractCommand(elevator));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristUpCommand(wrist)
                ), true
        ).whenInactive(
                new WristStopCommand(wrist)
        );

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >  0.5).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristDownCommand(wrist)
                ), true
        ).whenInactive(
                new WristStopCommand(wrist)
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileActiveContinuous(
                new SequentialCommandGroup(
                        new ClimberUpCommand(climber)
                ), true
        ).whenInactive(
                new ClimberStopCommand(climber)
        );

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >  0.5).whileActiveContinuous(
                new ParallelCommandGroup(
                        new ClimberDownCommand(climber)
                ), true
        ).whenInactive(
                new ClimberStopCommand(climber)
        );


        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(new GrabberPickupToggleCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(new GrabberDropToggleCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.BACK).toggleWhenPressed(
            new ParallelCommandGroup(
                new ElevatorRetractCommand(elevator)),
                new WormResetCommand(worm).interruptOn(() -> worm.getAngle() <= 10 )
            );
    }

    public void testingConfigV3() {
        //Y button scoring position in top bucket
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
            new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 65),
            new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() >= 34)
        ));

        //X button pick up position -- close to ground
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SequentialCommandGroup(
                new ElevatorV2RetractCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.isRetracted()),
                new WormSetPowerCommand(worm, -1).interruptOn(() -> worm.getAngle() <= -5)
        ));

        //right bumper wrist up
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristUpCommand(wrist)
                ), true
        ).whenInactive(
                new WristStopCommand(wrist)
        );

        //right trigger wrist down
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >  0.5).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristDownCommand(wrist)
                ), true
        ).whenInactive(
                new WristStopCommand(wrist)
        );

        //start button set to score top bar for specimen
        driver.getGamepadButton(GamepadKeys.Button.START).whenPressed(new SequentialCommandGroup(
                new WormSetPowerCommand(worm, 1).interruptOn(() -> worm.getAngle() > 23.00),
                new WormSetPowerCommand(worm, 0.2).interruptOn(() -> worm.getAngle() > 27.5), //final target: 28.5
                new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.getDistanceInInches() >= 11.0),
                new ElevatorV2ExtendCommand(elevatorV2, 0.5).interruptOn(() -> elevatorV2.getDistanceInInches() >= 12.8),
                new WristSetAngleCommand(wrist, 132.0) //132.0
        ));

        //control the elevator with the dpad
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new ElevatorV2ExtendCommand(elevatorV2, 1));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new ElevatorV2RetractCommand(elevatorV2, 1));

        //control the input and output of the grabber
        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(new GrabberPickupToggleCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(new GrabberDropToggleCommand(grabber));

        //begin final ascent (retract elevator and worm)
        driver.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ParallelCommandGroup(
                new RunCommand(() -> worm.lower(-1), worm),
                new RunCommand(() -> elevatorV2.setPower(-1), elevatorV2)
        ));

        //right joystick controls worm height
        new Trigger(() -> 0.1 < Math.abs(driver.getRightY())).whenActive(
                new RunCommand(() -> worm.setPower(-driver.getRightY()), worm)
        );

        //set the driving command
        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );
    }

    @Override
    public void run() {
        super.run();
        elevatorDistance = elevatorV2.getDistance();
        telemetry.addData("elevatorDistance", elevatorDistance);
        telemetry.update();

        elevatorV2.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go
        worm.SetElevatorDistanceInInches(elevatorV2.getHorizontalExtension());
    }
}
