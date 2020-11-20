package org.wolfcorp.ultimategoal.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.drive.Drivetrain;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="Autonomous Test Spline", group = "drive")
public class AutonomousTestWeirdSpline extends LinearOpMode {
    public static double DISTANCE = 10; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);

        Trajectory trajectory = drive.from(new Pose2d())
                .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.follow(trajectory);
    }
}
