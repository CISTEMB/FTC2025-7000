package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.BeltwaySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotorsSubsystem;

public class StopLauncherMotorsCommand extends InstantCommand {

    public StopLauncherMotorsCommand(LauncherMotorsSubsystem motors, BeltwaySubsystem beltway) {
        super(() -> {
            motors.stop();
            beltway.stop();
        }, motors, beltway);
    }
}
