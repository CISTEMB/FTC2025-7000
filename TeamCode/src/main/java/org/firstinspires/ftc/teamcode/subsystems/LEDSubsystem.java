package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LEDSubsystem extends SubsystemBase {
    private final Servo led1;
    private final Servo led2;
    private final Telemetry t;
    public enum Color {
        OFF,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE,
    }
    public LEDSubsystem(HardwareMap hardwareMap, Telemetry t) {
        led1 = hardwareMap.get(Servo.class, "RGBled1");
        led2 = hardwareMap.get(Servo.class, "RGBled2");
        this.t = t;
    }
    public void displayColor(Color color) {
        double rawColor;
        switch (color) {
            case OFF:
                rawColor = 0.0;
                break;
            case RED:
                rawColor = 0.277;
                break;
            case ORANGE:
                rawColor = 0.333;
                break;
            case YELLOW:
                rawColor = 0.388;
                break;
            case SAGE:
                rawColor = 0.444;
                break;
            case GREEN:
                rawColor = 0.5;
                break;
            case AZURE:
                rawColor = 0.555;
                break;
            case BLUE:
                rawColor = 0.611;
                break;
            case INDIGO:
                rawColor = 0.666;
                break;
            case VIOLET:
                rawColor = 0.722;
                break;
            case WHITE:
            default:
                rawColor = 1.0;
        }
        led1.setPosition(rawColor);
        led2.setPosition(rawColor);
    }
}
