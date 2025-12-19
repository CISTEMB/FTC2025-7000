package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PickupCommand extends CommandBase {

    private IntakeSubsystem intake;

    public PickupCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.forward();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
