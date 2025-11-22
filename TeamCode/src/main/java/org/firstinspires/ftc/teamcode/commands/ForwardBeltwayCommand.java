package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Beltway;

public class ForwardBeltwayCommand extends CommandBase {

    private Beltway beltway;

    public ForwardBeltwayCommand(Beltway b) {
        this.beltway = b;
        addRequirements(b);
    }

    @Override
    public void execute() { beltway.forward(); }

    @Override
    public void end(boolean interrupted) { beltway.stop(); }
}
