package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LifterSubsystem extends SubsystemBase {

    public Servo servo;
    private AnalogInput angleInput;
    private NavigationSubsystem navigation;
    private Telemetry tm;
    private final int increment = 1;
    private final int minPosition = 0;
    private final int maxPosition = 4;
    private double currentPosition = 0;

    private final InterpLUT lifterPosList = new InterpLUT();

    public void increasePosition() {
        if (currentPosition < maxPosition) {
            currentPosition += increment;
            servo.setPosition(lifterPosList.get(currentPosition));
        }
    }


    public LifterSubsystem(HardwareMap hardwareMap, Telemetry telemetry, NavigationSubsystem navigation) {
        tm = telemetry;
        servo = hardwareMap.get(Servo.class, "lifterServo");
        this.navigation = navigation;

        lifterPosList.add(-0.001, 1.0);
        lifterPosList.add(0.0, 1.0);
        lifterPosList.add(1.0, 0.94);
        lifterPosList.add(2.0, 0.88);
        lifterPosList.add(3.0, 0.82);
        lifterPosList.add(4.0, 0.76);
        lifterPosList.add(4.001, 0.7);

        lifterPosList.createLUT();
        this.setServoPosition(0.0); //level out the servo
    }

    public void setServoPosition(double position) {
        servo.setPosition(position);
        tm.addData("setServoPosition", position);
    }

    public void setPosition(double position) {
        if (position < minPosition ) {
            tm.addData("position under minimum requested: ", position);
            tm.update();
            return;
        }

        if (position > maxPosition) {
            tm.addData("position over maximum requested: ", position);
            tm.update();
            return;
        }
        this.currentPosition = position;
        servo.setPosition(lifterPosList.get(currentPosition));
    }

    public double getPosition() {
        return currentPosition;
    }



    public void decreasePosition() {
        if (currentPosition > minPosition) {
            currentPosition -= increment;
            servo.setPosition(lifterPosList.get(currentPosition));
        }
    }

    public double getAngle() {
        // TODO: Uncomment when angle sensor is connected
        // return angleInput.getVoltage() / angleInput.getMaxVoltage();
        return 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        Double pos = navigation.getPosition();
        if (pos == null) {
            return;
        }
        
        if (navigation.hasSeenTag() && pos > 0.0) {
            setPosition(pos);
        }
    }
}
