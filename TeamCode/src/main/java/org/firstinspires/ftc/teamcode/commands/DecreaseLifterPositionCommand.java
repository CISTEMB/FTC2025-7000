package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Lifter;

public class DecreaseLifterPositionCommand extends InstantCommand {

    public DecreaseLifterPositionCommand(Lifter lifter) {
        super(lifter::decreasePosition, lifter);
    }
}
