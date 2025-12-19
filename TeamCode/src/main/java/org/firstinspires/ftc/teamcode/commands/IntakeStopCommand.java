package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends CommandBase {

    private IntakeSubsystem intake;

    public IntakeStopCommand(IntakeSubsystem i) {
        this.intake = i;
        addRequirements(i);
    }

    @Override
    public void execute() {
        intake.stop();
    }
}
