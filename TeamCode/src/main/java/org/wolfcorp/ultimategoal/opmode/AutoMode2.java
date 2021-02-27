package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;
import org.wolfcorp.ultimategoal.vision.BrightOpenCVZoneChooser;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.ZoneChooser;

@Autonomous(name="WCAuto 2.0")
public class AutoMode2 extends LinearOpMode {
    protected ZoneChooser chooser;

    protected Pose2d initialPose = new Pose2d(-65, 48, Math.toRadians(0));

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, true);
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

        chooser = new BrightOpenCVZoneChooser();
        chooser.init(hardwareMap, telemetry);
        Pose2d targetZoneA;
        Pose2d targetZoneB;

        waitForStart();

        Target target = chooser.getTarget();
        telemetry.addData("target", target);
        telemetry.update();
        switch (target) {
            case A:
                targetZoneA = new Pose2d(12, 56, Math.toRadians(0));
                targetZoneB = new Pose2d(14, 51, Math.toRadians(0));
                break;
            case B:
                targetZoneA = new Pose2d(22, 34, Math.toRadians(-90));
                targetZoneB = new Pose2d(22, 36, Math.toRadians(-90));
                break;
            default:
            case UNSET:
            case C:
                targetZoneA = new Pose2d(46, 54, Math.toRadians(-45));
                targetZoneB = new Pose2d(49, 50, Math.toRadians(-45));
                break;
        }

        telemetry.addData("target", target);
        telemetry.addData("AX", targetZoneA.getX());
        telemetry.addData("AY", targetZoneA.getY());
        telemetry.addData("BX", targetZoneB.getX());
        telemetry.addData("BY", targetZoneB.getY());
        telemetry.update();

        drive.setPoseEstimate(initialPose);

        // Move and shoot
        Trajectory traj1 = drive
                .from(initialPose)
                .lineToLinearHeading(new Pose2d(-14, 52, Math.toRadians(-10)))
                .build();

        // Shoot -> drop wobble goal
        // furthest zone : new Pose2d(48, 56, Math.toRadians(90)
        Trajectory traj2 = drive.from(traj1.end())
                .lineToLinearHeading(targetZoneA)
                .build();

        // Drop -> pick up rings
        Trajectory traj3 = drive.from(traj2.end())
                .lineToLinearHeading(new Pose2d(0, 36, Math.toRadians(0)))
                .build();

        Trajectory traj3Z = drive.from(traj3.end())
                .lineToLinearHeading(new Pose2d(-12, 37, Math.toRadians(3)))
                .build();

        Trajectory traj3a = drive.from(traj3Z.end()).back(10)
                .addTemporalMarker(0, scorer::stopperClose)
                .build();

        Trajectory traj3b = drive.from(traj3a.end(),30,30).back(15)
                .addTemporalMarker(0, scorer::stopperOpen)
                .addTemporalMarker(0.34, scorer::stopperClose)
                .addTemporalMarker(0.34, () -> {
                    scorer.outtakeOn(-300);
                })
                .build();
/*
        Trajectory traj3c = drive.from(traj3b.end()).forward(12).build();

        Trajectory traj3d = drive.from(traj3c.end()).back(20).build();

        Trajectory traj3e = drive.from(traj3d.end()).forward(10).build();*/

        // Path to near wobble goal
        Trajectory traj4 = drive.from(new Pose2d(traj3b.end().getX(), traj3b.end().getY(), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-56, 44, Math.toRadians(180)))
                .build();

        // Path to wobble goal
        Trajectory traj5 = drive.from(traj4.end())
                .lineToLinearHeading(
                        new Pose2d(-56, 37, Math.toRadians(180))
                )
                .build();

        // Path to Zones
        Trajectory traj6 = drive.from(traj5.end())
                .lineToLinearHeading(targetZoneB)
                .build();

        // Path to Park
        Trajectory traj7 = drive.from(traj6.end())
                .lineToLinearHeading(new Pose2d(12, 36,0))
                .build();

        scorer.gripperClose();
        scorer.openRelease();

        // Move and shoot (high goal)
        scorer.outtakeOn();
        drive.follow(traj1);
        for (int i = 0; i < 3; i++) {
            sleep(300);
            scorer.stopperClose();
            sleep(200);
            scorer.stopperOpen();
        }
        scorer.outtakeOff();

        // Drop the wobble goal (in Zone C for now)
        drive.follow(traj2);
        scorer.armOut();
        scorer.gripperOpen();
        scorer.armIn();

        // Go back to pick up the rings
        scorer.intakeSlow();
        scorer.outtakeOn(-300);
        drive.follow(traj3);
        drive.follow(traj3Z);
        drive.follow(traj3a);
        drive.follow(traj3b);
        for (int i = 0; i < 6; i++) {
            sleep(300);
            scorer.stopperOpen();
            sleep(200);
            scorer.stopperClose();
        }
        scorer.outtakeOff();

        // Drive near wobble goal
        drive.turn(Math.toRadians(180));
        scorer.intakeOff();
        drive.updatePoseEstimate();
        drive.follow(traj4);

        // Lower wobble arm
        scorer.gripperOpen();
        scorer.armOut();

        // Drive to wobble goal
        drive.follow(traj5);

        // Grab wobble goal
        sleep(50);
        scorer.gripperClose();

        // Drive to Zone
        drive.follow(traj6);

        // Dropping wobble goal
        scorer.gripperOpen();
        // TODO: Make this a temporal marker
        //scorer.armIn();

        // Parking maneuver
        drive.follow(traj7);
        drive.turnTo(0);
    }
}
