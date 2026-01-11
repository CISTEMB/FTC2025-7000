package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorMotorsSubsystem;

public class ElevatorDownCommand extends CommandBase {

    private ElevatorMotorsSubsystem elevator;

    public ElevatorDownCommand(ElevatorMotorsSubsystem e) {
        this.elevator = e;
        addRequirements(e);
    }

    @Override
    public void execute() { elevator.goDown(); }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}