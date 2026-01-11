package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorMotorsSubsystem;

public class ElevatorUpCommand extends CommandBase {

    private ElevatorMotorsSubsystem elevator;

    public ElevatorUpCommand(ElevatorMotorsSubsystem e) {
        this.elevator = e;
        addRequirements(e);
    }

    @Override
    public void execute() { elevator.goUp(); }


    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}