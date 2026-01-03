package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LAZER;

public class AutoPickupCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final LAZER lazer;
    private final Telemetry tm;
    private final IntakeSubsystem intake;
    private final BeltwaySubsystem beltway;

    public AutoPickupCommand(DriveSubsystem drive, IntakeSubsystem intake, BeltwaySubsystem beltway, LAZER lazer, Telemetry telemetry) {
        this.drive = drive;
        this.lazer = lazer;
        this.tm = telemetry;
        this.intake = intake;
        this.beltway = beltway;

        addRequirements(drive, lazer);
    }

    @Override
    public void initialize() {
        intake.forward();
        drive.arcadeDrive(-1.0, 0.0, 0.0, false);
    }

    @Override
    public void execute() {

    }
}
