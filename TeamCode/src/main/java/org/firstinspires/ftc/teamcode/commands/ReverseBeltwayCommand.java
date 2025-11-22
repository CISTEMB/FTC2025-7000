package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Beltway;

public class ReverseBeltwayCommand extends CommandBase {

    private Beltway beltway;

    public ReverseBeltwayCommand(Beltway b) {
        this.beltway = b;
        addRequirements(b);
    }

    @Override
    public void execute() { beltway.reverse(); }

    @Override
    public void end(boolean interrupted) { beltway.stop(); }
}