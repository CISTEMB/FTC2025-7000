package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorMotorsSubsystem;

public class ElevatorStopCommand extends CommandBase {

    private ElevatorMotorsSubsystem elevator;

    public ElevatorStopCommand(ElevatorMotorsSubsystem e) {
        this.elevator = e;
        addRequirements(e);
    }

    @Override
    public void execute() { elevator.stop(); }
}