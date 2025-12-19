package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class DecreaseLifterPositionCommand extends InstantCommand {

    public DecreaseLifterPositionCommand(LifterSubsystem lifter) {
        super(lifter::decreasePosition, lifter);
    }
}
