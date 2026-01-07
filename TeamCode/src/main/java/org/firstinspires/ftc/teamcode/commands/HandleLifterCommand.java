package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NavigationSubsystem;

public class HandleLifterCommand extends CommandBase {
    private LifterSubsystem lifter;
    private NavigationSubsystem navigation;
    public HandleLifterCommand(LifterSubsystem lifter, NavigationSubsystem navigation) {
        this.lifter = lifter;
        this.navigation = navigation;

        addRequirements(lifter, navigation);
    }

    @Override
    public void execute() {
        Double pos = navigation.getPosition();
        if (pos == null) {
            return;
        }

        if (navigation.hasSeenTag() && pos > 0.0) {
            lifter.setPosition(pos);
        }
    }
}
