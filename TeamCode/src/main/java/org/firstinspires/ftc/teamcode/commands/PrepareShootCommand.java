package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LauncherMotors;

public class PrepareShootCommand extends InstantCommand {

    public PrepareShootCommand(LauncherMotors motors) {
        super(() -> motors.prepareShoot(), motors);
    }
}
