package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
    protected Pose2d initialPose = new Pose2d();
    protected StartingPosition sp = StartingPosition.UNSET;

    public abstract Vector2d getTargetPos(Target t);

    @Override
    public void runOpMode() {
        // To change the vision algorithm, change the constructor used
        chooser = new TFODZoneChooser();
        chooser.init(hardwareMap, telemetry);
        waitForStart();
        Vector2d zonePos = getTargetPos(chooser.getTarget());
        chooser.stop();
        // TODO: if we can't ensure the exact position where we place the robot
        //   we might need to consider ramming against the wall
        drive.follow(drive
                .from(initialPose)
                .splineTo(zonePos, initialPose.getHeading())
                .build());
        // TODO: drop goal
        // TODO: maybe reset by banging against wall
        // TODO: power shot one-by-one
        // TODO: depending on vision result, pick up starter stack
        //  and aim for the high goal
        // TODO: park
    }
}
