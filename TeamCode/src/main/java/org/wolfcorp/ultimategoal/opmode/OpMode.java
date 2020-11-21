package org.wolfcorp.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.drive.Drivetrain;

public abstract class OpMode extends LinearOpMode {
    protected Drivetrain drive = new Drivetrain(hardwareMap);
}
