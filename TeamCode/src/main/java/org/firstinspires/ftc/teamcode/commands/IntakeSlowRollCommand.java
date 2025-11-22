package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSlowRollCommand extends CommandBase {

    private Intake intake;

    public IntakeSlowRollCommand(Intake i) {
        this.intake = i;
        addRequirements(i);
    }

    @Override
    public void execute() {
        intake.slowRoll();
    }


    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
