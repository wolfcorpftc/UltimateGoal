package org.wolfcorp.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.robot.Drivetrain;

public abstract class OpMode extends LinearOpMode {
    protected Drivetrain drive = new Drivetrain(hardwareMap);

    void shoot() {}

    void startIntake(){}
    void stopIntake(){}
}
