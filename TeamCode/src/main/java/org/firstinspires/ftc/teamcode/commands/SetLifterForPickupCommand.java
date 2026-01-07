package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class SetLifterForPickupCommand extends InstantCommand {
    private LifterSubsystem lifter;

    public SetLifterForPickupCommand(LifterSubsystem lifter) {
        this.lifter = lifter;
        addRequirements(lifter);
    }


    @Override
    public void execute() { lifter.setServoPosition(0.0); }
}
