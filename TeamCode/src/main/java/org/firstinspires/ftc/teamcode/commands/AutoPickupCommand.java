package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Beltway;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LAZER;

public class AutoPickupCommand extends CommandBase {
    private final Drive drive;
    private final LAZER lazer;
    private final Telemetry tm;
    private final Intake intake;
    private final Beltway beltway;

    public AutoPickupCommand(Drive drive, Intake intake, Beltway beltway, LAZER lazer, Telemetry telemetry) {
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
