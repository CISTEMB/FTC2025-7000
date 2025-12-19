package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;

public class StopBeltwayCommand extends CommandBase {

    private BeltwaySubsystem beltway;

    public StopBeltwayCommand(BeltwaySubsystem b) {
        this.beltway = b;
        addRequirements(b);
    }

    @Override
    public void execute() { beltway.stop(); }

}