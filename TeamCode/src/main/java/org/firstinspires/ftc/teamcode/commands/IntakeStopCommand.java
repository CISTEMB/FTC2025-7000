package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeStopCommand extends CommandBase {

    private Intake intake;

    public IntakeStopCommand(Intake i) {
        this.intake = i;
        addRequirements(i);
    }

    @Override
    public void execute() {
        intake.stop();
    }
}
