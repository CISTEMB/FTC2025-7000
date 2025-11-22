package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Beltway;
import org.firstinspires.ftc.teamcode.subsystems.LauncherMotors;

public class StopLauncherMotorsCommand extends InstantCommand {

    public StopLauncherMotorsCommand(LauncherMotors motors, Beltway beltway) {
        super(() -> {
            motors.stop();
            beltway.stop();
        }, motors, beltway);
    }
}
