package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LAZER extends SubsystemBase {
    private final Telemetry tm;

    public final DistanceSensor left;
    public final DistanceSensor right;

    public LAZER(HardwareMap hardwareMap, Telemetry telemetry) {
        tm = telemetry;

        left = hardwareMap.get(DistanceSensor.class, "leftDistance");
        right = hardwareMap.get(DistanceSensor.class, "rightDistance");
    }


}
