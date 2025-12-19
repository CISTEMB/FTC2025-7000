package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;

public class ReverseBeltwayCommand extends CommandBase {

    private BeltwaySubsystem beltway;

    public ReverseBeltwayCommand(BeltwaySubsystem b) {
        this.beltway = b;
        addRequirements(b);
    }

    @Override
    public void execute() { beltway.reverse(); }

    @Override
    public void end(boolean interrupted) { beltway.stop(); }
}