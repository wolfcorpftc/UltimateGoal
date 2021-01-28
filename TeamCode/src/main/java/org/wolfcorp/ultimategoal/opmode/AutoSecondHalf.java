package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;
import org.wolfcorp.ultimategoal.vision.StartingPosition;
import org.wolfcorp.ultimategoal.vision.TFODZoneChooser;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.ZoneChooser;

// TODO: add actual autonomous opmodes
@Autonomous(name="Hello")
public class AutoSecondHalf extends LinearOpMode {

    @Override
    public void runOpMode() {

        Drivetrain drive = new Drivetrain(hardwareMap);
        Scorer scorer = new Scorer(hardwareMap);

        //top left of field is +x +y, bottom right is -x -y
        //horizontal axis is y, vertical axis is x
        //0,0 is center of field
        //Rotation of 0 is straight up, uses radians

        Pose2d startPose = new Pose2d(-24, -36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.from(startPose)
                .splineTo(new Vector2d(-48, -36), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.from(traj1.end())
                .splineTo(new Vector2d(-48, -30), Math.toRadians(0))
                .build();

        Trajectory traj3 = drive.from(traj2.end())
                .splineTo(new Vector2d(48, -60), Math.toRadians(-90))
                .build();

        Trajectory traj4 = drive.from(traj3.end())
                .splineTo(new Vector2d(12, -60), Math.toRadians(0))
                .build();

        waitForStart();

        drive.follow(traj1);

        scorer.gripperOpen();

        scorer.armDown();

        drive.follow(traj2);

        scorer.gripperClose();

        scorer.armUp();

        drive.follow(traj3);

        scorer.armDown();

        scorer.gripperOpen();

        drive.follow(traj4);


        // TODO: drop goal
        // TODO: maybe reset by banging against wall
        // TODO: power shot one-by-one
        // TODO: depending on vision result, pick up starter stack
        //  and aim for the high goal
        // TODO: park
    }
}
