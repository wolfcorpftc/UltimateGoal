package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ultimategoal.robot.Scorer;
import org.wolfcorp.ultimategoal.vision.TFODZoneChooser;
import org.wolfcorp.ultimategoal.vision.StartingPosition;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.ZoneChooser;

// TODO: add actual autonomous opmodes
@Autonomous(name="WCAuto")
public abstract class CurrentAuto extends OpMode {
    protected ZoneChooser chooser;

    // configration for specific starting positions / plans
    // TODO: figure out the four initialPoses
    protected Pose2d initialPose = new Pose2d(-65, 48, Math.toRadians(0));
    protected StartingPosition sp = StartingPosition.UNSET;

    public abstract Vector2d getTargetPos(Target t);

    @Override
    public void runOpMode() {
        // To change the vision algorithm, change the constructor used
        chooser = new TFODZoneChooser();
        chooser.init(hardwareMap, telemetry);
        waitForStart();
        Scorer scorer = new Scorer(hardwareMap);
        Vector2d zonePos = getTargetPos(chooser.getTarget());
        chooser.stop();
        // TODO: if we can't ensure the exact position where we place the robot
        //   we might need to consider ramming against the wall
        Trajectory traj1 = drive
                .from(initialPose)
                .lineToLinearHeading(new Pose2d(-18, 54, Math.toRadians(-30)))
                .addDisplacementMarker(() -> {
                    scorer.toggleOuttake(true, 0);
                })
                .build();
        Trajectory traj2 = drive.from(traj1.end())
                .splineToSplineHeading(new Pose2d(48, 56, Math.toRadians(90)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scorer.gripperOpen();
                    scorer.armUp();
                    // FIXME: figure out how to properly close it
                })
                .build();
        Trajectory traj3 = drive.from(traj2.end())
                .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(-30)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scorer.toggleIntake(true, 1, 200);
                    sleep(500);
                    scorer.toggleIntake(true, 1, 200);
                })
                .build();

        drive.follow(traj1);
        drive.follow(traj2);
        drive.follow(traj3);

        // TODO: drop goal
        // TODO: maybe reset by banging against wall
        // TODO: power shot one-by-one
        // TODO: depending on vision result, pick up starter stack
        //  and aim for the high goal
        // TODO: park
    }
}
