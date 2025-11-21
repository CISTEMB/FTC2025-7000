package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ReverseIntakeCommand extends CommandBase {

    private Intake intake;

    public ReverseIntakeCommand(Intake i) {
        this.intake = i;
        addRequirements(i);
    }

    @Override
    public void execute() {
        intake.reverse();
    }


    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}