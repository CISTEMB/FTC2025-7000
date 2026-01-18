package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LAZER;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class LaserLightShowCommand extends CommandBase {
    private final LAZER lazer;
    private final LEDSubsystem led;

    public LaserLightShowCommand(LAZER laser, LEDSubsystem led) {
        this.lazer = laser;
        this.led = led;
        addRequirements(led);
    }

    @Override
    public void execute()
    {
        double leftDistance = lazer.getLeftDistance();// .getDistance(DistanceUnit.CM); // laser distance is 10 cm
        double rightDistance = lazer.getRightDistance(); //(DistanceUnit.CM);

        //these are configured backwards on purpose
        double rightColor = map(leftDistance, 0.0, 2.0, 0.388, 0.611);
        double leftColor = map(rightDistance, 0.0, 2.0, 0.388, 0.611);

        led.displayRawColor(leftColor, rightColor);
    }

    private static double map(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
        if (value > 2) { value = 2;}
        return outputMin + (outputMax - outputMin) * ((value - inputMin) / (inputMax - inputMin));
    }
}
