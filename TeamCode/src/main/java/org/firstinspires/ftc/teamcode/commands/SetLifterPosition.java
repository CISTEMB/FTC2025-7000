package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Lifter;

public class SetLifterPosition extends InstantCommand {

    public SetLifterPosition(double position, Lifter lifter) {
        super(() -> lifter.servo.setPosition(position), lifter);
    }
}
