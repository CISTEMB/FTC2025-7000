package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class SetLifterPositionCommand extends InstantCommand {
    private LifterSubsystem lifter;
    private double position;

    public SetLifterPositionCommand(double position, LifterSubsystem lifter) {
        this.lifter = lifter;
        this.position = position;
        addRequirements(lifter);
    }


    @Override
    public void execute() { lifter.setPosition(position); }
}
