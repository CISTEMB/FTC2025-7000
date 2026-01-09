package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;

public class HandleLauncherMotorsCommand extends CommandBase {
    private LauncherMotorsSubsystem launcherMotors;
    private NavigationSubsystem navigation;
    public HandleLauncherMotorsCommand(LauncherMotorsSubsystem launcherMotors, NavigationSubsystem navigation) {
        this.launcherMotors = launcherMotors;
        this.navigation = navigation;

        addRequirements(launcherMotors, navigation);
    }

    @Override
    public void execute() {
        Double pos = navigation.getPosition();
        if (pos == null) {
            return;
        }

        if (navigation.hasSeenTag() && pos > 0.0) {
            launcherMotors.setSpeedBasedOnLifterPosition(pos);
        }
    }
}
