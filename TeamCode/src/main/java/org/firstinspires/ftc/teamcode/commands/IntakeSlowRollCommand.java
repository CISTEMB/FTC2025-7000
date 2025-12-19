package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeSlowRollCommand extends CommandBase {

    private IntakeSubsystem intake;

    public IntakeSlowRollCommand(IntakeSubsystem i) {
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
