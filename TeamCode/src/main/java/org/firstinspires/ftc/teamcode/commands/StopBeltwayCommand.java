package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Beltway;

public class StopBeltwayCommand extends CommandBase {

    private Beltway beltway;

    public StopBeltwayCommand(Beltway b) {
        this.beltway = b;
        addRequirements(b);
    }

    @Override
    public void execute() { beltway.stop(); }

}