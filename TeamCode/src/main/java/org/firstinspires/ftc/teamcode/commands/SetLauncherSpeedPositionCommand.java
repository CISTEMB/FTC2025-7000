package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

public class SetLauncherSpeedPositionCommand extends InstantCommand {
    private LauncherMotorsSubsystem launcher;
    private double position;

    public SetLauncherSpeedPositionCommand(double position, LauncherMotorsSubsystem launcher) {
        this.launcher = launcher;
        this.position = position;
        addRequirements(launcher);
    }


    @Override
    public void execute() { launcher.setSpeedBasedOnLifterPosition(position); }
}
