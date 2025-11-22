package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Lifter;

public class IncreaseLifterPositionCommand extends InstantCommand {

    public IncreaseLifterPositionCommand(Lifter lifter) {
        super(() -> lifter.increasePosition(), lifter);
    }
}
