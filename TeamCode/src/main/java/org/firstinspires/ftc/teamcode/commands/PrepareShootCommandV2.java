package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.LauncherMotors;
import org.firstinspires.ftc.teamcode.subsystems.Lifter;

public class PrepareShootCommandV2 extends InstantCommand {

    public PrepareShootCommandV2(LauncherMotors motors, Lifter lifter) {
        super(() -> motors.setSpeedBasedOnLifterPosition(lifter.getPosition()), motors);
    }
}
