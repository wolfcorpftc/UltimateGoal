package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ultimategoal.robot.Scorer;

@Autonomous(name="WCAuto")
public class AutoMode extends OpMode {
    //protected ZoneChooser chooser;

    protected Pose2d initialPose = new Pose2d(-65, 48, Math.toRadians(0));

    @Override
    public void runOpMode() {
        // To change the vision algorithm, change the constructor used
        //chooser = new TFODZoneChooser();
        //chooser.init(hardwareMap, telemetry);
        Scorer scorer = new Scorer(hardwareMap);

        //Vector2d zonePos = getTargetPos(chooser.getTarget());
        //chooser.stop();

        // top left of field is +x +y, bottom right is -x -y
        // horizontal axis is y, vertical axis is x
        // 0,0 is center of field
        // Rotation of 0 is straight up, uses radians
        // Start -> shooting position

        drive.setPoseEstimate(initialPose);

        Trajectory traj1 = drive
                .from(initialPose)
                .lineToLinearHeading(new Pose2d(-18, 54, Math.toRadians(-30)))
                .build();

        // Shoot -> drop wobble goal
        // furthest zone : new Pose2d(48, 56, Math.toRadians(90)
        Trajectory traj2 = drive.from(traj1.end())
                .splineToSplineHeading(new Pose2d(48, 56, Math.toRadians(90)), Math.toRadians(0))
                .build();

        // Drop -> pick up rings
        Trajectory traj3 = drive.from(traj2.end())
                .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(-30)), Math.toRadians(0))
                .build();

        // Path to near wobble goal
        Trajectory traj4 = drive.from(traj3.end())
                .splineTo(new Vector2d(-24, 10), Math.toRadians(0))
                .build();

        // Path to nearer wobble goal
        Trajectory traj5 = drive.from(traj4.end())
                .splineTo(new Vector2d(-48, 10), Math.toRadians(0))
                .build();

        // Path to wobble goal
        Trajectory traj6 = drive.from(traj5.end())
                .splineTo(new Vector2d(-48, 22), Math.toRadians(0))
                .build();

        // Path to Zone C
        Trajectory traj7 = drive.from(traj6.end())
                .splineTo(new Vector2d(48, 60), Math.toRadians(90))
                .build();

        // Path to Park
        Trajectory traj8 = drive.from(traj7.end())
                .splineTo(new Vector2d(12, 60), Math.toRadians(0))
                .build();

        waitForStart();

        // Move a bit and shoot (high goal)
        drive.follow(traj1);
        scorer.outtake(500);

        // Drop the wobble goal (in Zone C for now)
        drive.follow(traj2);
        scorer.gripperOpen();
        scorer.armUp();

        // Go back to pick up the rings
        drive.follow(traj3);
        scorer.intake(500);

        // Drive near wobble goal
        drive.follow(traj4);
        drive.follow(traj5);

        // Lower wobble arm
        scorer.gripperOpen();
        scorer.armDown();

        // Drive to wobble goal
        drive.follow(traj6);

        // Grab wobble goal
        scorer.gripperClose();
        scorer.armUp();

        // Drive to Zone C
        drive.follow(traj7);

        // Dropping wobble goal
        scorer.armDown();
        scorer.gripperOpen();
        scorer.armUp();

        // Parking maneuver
        drive.follow(traj8);
    }
}
