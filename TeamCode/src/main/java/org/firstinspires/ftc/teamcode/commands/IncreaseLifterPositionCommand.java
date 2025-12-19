package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;


import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class IncreaseLifterPositionCommand extends InstantCommand {

    public IncreaseLifterPositionCommand(LifterSubsystem lifter) {
        super(lifter::increasePosition, lifter);
    }
}
