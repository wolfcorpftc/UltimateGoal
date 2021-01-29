package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@Autonomous(name="WCAuto")
public class AutoMode extends LinearOpMode {
    //protected ZoneChooser chooser;

    protected Pose2d initialPose = new Pose2d(-65, 48, Math.toRadians(0));

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);
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

        // Move and shoot
        Trajectory traj1 = drive
                .from(initialPose)
                .lineToLinearHeading(new Pose2d(-18, 54, Math.toRadians(-10)))
                .build();

        // Shoot -> drop wobble goal
        // furthest zone : new Pose2d(48, 56, Math.toRadians(90)
        Trajectory traj2 = drive.from(traj1.end())
                .lineToLinearHeading(new Pose2d(48, 56, Math.toRadians(-90)))
                .build();

        // Drop -> pick up rings
        Trajectory traj3 = drive.from(traj2.end())
                .lineToLinearHeading(new Pose2d(-13, 42, Math.toRadians(30)))
                .build();

        // Path to near wobble goal
        Trajectory traj4 = drive.from(traj3.end())
                .lineToLinearHeading(new Pose2d(-48, 42, Math.toRadians(180)))
                .build();

        // Path to wobble goal
        Trajectory traj5 = drive.from(traj4.end())
                .lineToLinearHeading(new Pose2d(-48, 38, Math.toRadians(180)))
                .build();

        // Path to Zone C
        Trajectory traj6 = drive.from(traj5.end())
                .lineToLinearHeading(new Pose2d(48, 60, Math.toRadians(-90)))
                .build();

        // Path to Park
        Trajectory traj7 = drive.from(traj6.end())
                .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(0)))
                .build();

        waitForStart();
        scorer.gripperClose();

        // Move and shoot (high goal)
        drive.follow(traj1);
        scorer.outtakeOn();
        for (int i = 0; i < 3; i++) {
            sleep(300);
            scorer.stopperOpen();
            sleep(200);
            scorer.stopperClose();
        }
        scorer.outtakeOff();

        // Drop the wobble goal (in Zone C for now)
        drive.follow(traj2);
        scorer.armUp();
        scorer.gripperOpen();

        // Go back to pick up the rings
        drive.follow(traj3);
        scorer.intake(500);

        // Drive near wobble goal
        drive.follow(traj4);

        // Lower wobble arm
        scorer.armDown();
        scorer.gripperOpen();

        // Drive to wobble goal
        drive.follow(traj5);

        // Grab wobble goal
        scorer.gripperClose();
        scorer.armUp();

        // Drive to Zone C
        drive.follow(traj6);

        // Dropping wobble goal
        scorer.armDown();
        scorer.gripperOpen();
        scorer.armUp();

        // Parking maneuver
        drive.follow(traj7);
    }
}
