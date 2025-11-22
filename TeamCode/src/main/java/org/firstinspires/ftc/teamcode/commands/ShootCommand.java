package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Beltway;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ShootCommand extends CommandBase {

    private Beltway beltway;
    private Intake intake;

    public ShootCommand(Beltway beltway, Intake intake) {
        this.beltway = beltway;
        this.intake = intake;
        addRequirements(beltway, intake);
    }

    @Override
    public void execute() {
        beltway.forward();
        intake.slowRoll();
    }

    @Override
    public void end(boolean interrupted) {
        beltway.stop();
        intake.stop();
    }
}
