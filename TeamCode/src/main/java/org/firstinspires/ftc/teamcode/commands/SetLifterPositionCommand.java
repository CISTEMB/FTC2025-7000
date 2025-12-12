package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Lifter;

public class SetLifterPositionCommand extends InstantCommand {
    private Lifter lifter;
    private int position;

    public SetLifterPositionCommand(int position, Lifter lifter) {
        this.lifter = lifter;
        this.position = position;
        addRequirements(lifter);
    }


    @Override
    public void execute() { lifter.setPosition(position); }
}
