package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ReverseIntakeCommand extends CommandBase {

    private IntakeSubsystem intake;

    public ReverseIntakeCommand(IntakeSubsystem i) {
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