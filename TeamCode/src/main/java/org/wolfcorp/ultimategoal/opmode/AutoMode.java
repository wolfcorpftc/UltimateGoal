package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wolfcorp.ultimategoal.robot.DriveConstants;
import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;
import org.wolfcorp.ultimategoal.vision.TFODZoneChooser;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.ZoneChooser;

import java.util.Arrays;

@Autonomous(name="WCAuto")
public class AutoMode extends LinearOpMode {
    protected ZoneChooser chooser;

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

        chooser = new TFODZoneChooser();
        chooser.init(hardwareMap, telemetry);
        Pose2d targetZoneA;
        Pose2d targetZoneB;

        waitForStart();

        Target target = chooser.getTarget();
        telemetry.addData("target", target);
        telemetry.update();
        switch (target) {
            default:
            case A:
            case UNSET:
                targetZoneA = new Pose2d(12, 56, Math.toRadians(0));
                targetZoneB = new Pose2d(10, 51, Math.toRadians(0));
                break;
            case B:
                targetZoneA = new Pose2d(36, 26, Math.toRadians(0));
                targetZoneB = new Pose2d(34, 24, Math.toRadians(0));
                break;
            case C:
                targetZoneA = new Pose2d(54, 54, Math.toRadians(0));
                targetZoneB = new Pose2d(60, 50, Math.toRadians(0));
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
                .lineToLinearHeading(new Pose2d(-14, 54, Math.toRadians(-6.5)))
                .build();

        // Shoot -> drop wobble goal
        // furthest zone : new Pose2d(48, 56, Math.toRadians(90)
        Trajectory traj2 = drive.from(traj1.end())
                .lineToLinearHeading(targetZoneA)
                .build();

        // Drop -> pick up rings
        Trajectory traj3 = drive.from(traj2.end())
                .lineToLinearHeading(new Pose2d(traj2.end().getX(), 36, Math.toRadians(0)))
                .build();

        Trajectory traj3a = drive.from(traj3.end())
                .lineToLinearHeading(new Pose2d(-13, traj3.end().getY(), Math.toRadians(0)))
                .build();

        Trajectory traj3b = drive.from(traj3a.end()).back(13).build();
/*
        Trajectory traj3c = drive.from(traj3b.end()).forward(12).build();

        Trajectory traj3d = drive.from(traj3c.end()).back(20).build();

        Trajectory traj3e = drive.from(traj3d.end()).forward(10).build();*/

        // Path to near wobble goal
        Trajectory traj4 = drive.from(new Pose2d(traj3b.end().getX(), traj3b.end().getY(), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-48.5, 44, Math.toRadians(180)))
                .build();

        // Path to wobble goal
        Trajectory traj5 = drive.from(traj4.end())
                .lineToLinearHeading(
                        new Pose2d(-48.5, 35.5, Math.toRadians(180))
                )
                .build();

        // Path to Zones
        Trajectory traj6 = drive.from(traj5.end())
                .lineToLinearHeading(targetZoneB)
                .build();

        // Path to Park
        Trajectory traj7 = drive.from(traj6.end())
                .lineToLinearHeading(new Pose2d(12, 36, traj6.end().getHeading()))
                .build();

        scorer.gripperClose();
        scorer.openRelease();

        // Move and shoot (high goal)
        scorer.outtakeOn();
        drive.follow(traj1);
        sleep(1000);
        for (int i = 0; i < 4; i++) {
            sleep(300);
            scorer.stopperOpen();
            sleep(200);
            scorer.stopperClose();
        }
        scorer.outtakeOff();

        // Drop the wobble goal (in Zone C for now)
        drive.follow(traj2);
        scorer.armOut();
        scorer.gripperOpen();
        scorer.armIn();

        // Go back to pick up the rings
        scorer.intakeOn();
        drive.follow(traj3);
        drive.follow(traj3a);
        drive.follow(traj3b);
        drive.turn(Math.toRadians(3));
        scorer.outtakeOn();
        sleep(1000);
        for (int i = 0; i < 4; i++) {
            sleep(300);
            scorer.stopperOpen();
            sleep(200);
            scorer.stopperClose();
        }
/*        drive.follow(traj3c);
        drive.follow(traj3d);
        scorer.intakeOff();
        sleep(1500);
        for (int i = 0; i < 4; i++) {
            sleep(300);
            scorer.stopperOpen();
            sleep(200);
            scorer.stopperClose();
        }*/
        scorer.intakeOff();
        scorer.outtakeOff();
        //drive.follow(traj3e);

        // Drive near wobble goal
        drive.turn(Math.toRadians(177));
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

        // Drive to Zone C
        // FIXME: INTAKE TOUCHES THE WALL
        drive.follow(traj6);

        // Dropping wobble goal
        scorer.gripperOpen();
        scorer.armIn();

        // Parking maneuver
        drive.follow(traj7);
    }
}
