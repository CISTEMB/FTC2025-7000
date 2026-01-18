package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;


public class LAZER extends SubsystemBase {
    private final Telemetry tm;
    public final AnalogInput left;
    public final AnalogInput right;



    public LAZER(HardwareMap hardwareMap, Telemetry telemetry) {
        left = hardwareMap.get(AnalogInput.class, "laserLeft");
        right = hardwareMap.get(AnalogInput.class, "laserRight");
        tm = telemetry;
    }

    public Double getLeftDistance() {
        return (left.getVoltage() / 3.3) * 10;
    }
    public Double getRightDistance() {
        return (right.getVoltage() / 3.3) * 10;
    }

    @Override
    public void periodic(){
//        super.periodic();
//
//        double leftDistance = (left.getVoltage() / 3.3) * 10;// .getDistance(DistanceUnit.CM); // laser distance is 10 cm
//        double rightDistance = (right.getVoltage() / 3.3) * 10; //(DistanceUnit.CM);
//
//        tm.addData("Left Laser Distance", leftDistance);
//        tm.addData("Right Laser Distance", rightDistance);
//
//        double leftColor = map(leftDistance, 0.0, 10.0, 0.388, 0.611);
//        double rightColor = map(rightDistance, 0.0, 10.0, 0.388, 0.611);
//
//        LED.displayRawColor(leftColor, rightColor);
    }

}
