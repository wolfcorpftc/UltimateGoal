package org.wolfcorp.ultimategoal.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface ZoneChooser {
    void init(HardwareMap hwMap, Telemetry telemetry);
    Target getTarget();
    void stop();
}
