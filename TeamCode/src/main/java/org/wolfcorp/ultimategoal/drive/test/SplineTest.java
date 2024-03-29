package org.wolfcorp.ultimategoal.drive.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.drive.Drivetrain;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.from(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.follow(traj);

        sleep(2000);

        drive.follow(
                drive.from(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
