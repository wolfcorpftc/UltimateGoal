package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;
import org.wolfcorp.ultimategoal.vision.BrightOpenCVZoneChooser;
import org.wolfcorp.ultimategoal.vision.Target;
import org.wolfcorp.ultimategoal.vision.ZoneChooser;

@Autonomous(name="WCAuto 2.0")
public class AutoMode2 extends LinearOpMode {
    protected ZoneChooser chooser;
    protected ElapsedTime outtakeTimeout = new ElapsedTime();

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

        Trajectory traj1 = drive
                .from(initialPose)
                .lineToLinearHeading(new Pose2d(-6, 52, Math.toRadians(-9)))
                .build();

        Trajectory traj3Z = drive.from(new Pose2d(0, 37, 0)) // traj3.end()
                .lineToLinearHeading(new Pose2d(-15, 37, Math.toRadians(0)))
                .build();

        Trajectory traj3a = drive.from(traj3Z.end(), 25, 25).back(7)
                .build();

        Trajectory traj3b = drive.from(traj3a.end(), 25, 25).back(6)
                .build();

        Trajectory traj3c = drive.from(traj3b.end(), 25, 25).back(9)
                .build();
/*
        Trajectory traj3c = drive.from(traj3b.end()).forward(12).build();

        Trajectory traj3d = drive.from(traj3c.end()).back(20).build();

        Trajectory traj3e = drive.from(traj3d.end()).forward(10).build();*/

        waitForStart();

        Target target = chooser.getTarget();
        chooser.stop();
        scorer.outtakeOn(-100);
        telemetry.addData("target", target);
        telemetry.update();
        switch (target) {
            case A:
                targetZoneA = new Pose2d(12, 50, Math.toRadians(0));
                targetZoneB = new Pose2d(14, 43, Math.toRadians(0));
                break;
            case B:
                targetZoneA = new Pose2d(22, 42, Math.toRadians(-90));
                targetZoneB = new Pose2d(18, 36, Math.toRadians(-90));
                break;
            default:
            case UNSET:
            case C:
                targetZoneA = new Pose2d(44, 56, Math.toRadians(-45));
                targetZoneB = new Pose2d(42, 56, Math.toRadians(-90));
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

        // Shoot -> drop wobble goal
        // furthest zone : new Pose2d(48, 56, Math.toRadians(90)
        Trajectory traj2 = drive.from(traj1.end())
                .lineToLinearHeading(targetZoneA)
                .build();

        // Drop -> pick up rings
        Trajectory traj3 = drive.from(traj2.end())
                .lineToLinearHeading(new Pose2d(0, 37, Math.toRadians(0))) // also modify traj3Z's from
                .build();

        // Path to near wobble goal
        Trajectory traj4 = drive.from(new Pose2d(traj3c.end().getX(), traj3c.end().getY(), Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-56.5 + (target != Target.C ? 1.5 : 0), 44, Math.toRadians(180)))
                .build();

        // Path to wobble goal
        Trajectory traj5 = drive.from(traj4.end())
                .lineToLinearHeading(
                        new Pose2d(-58 + (target != Target.C ? 3 : 0), 39 - (target == Target.A ? 2 : 0), Math.toRadians(180))
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

        // Move and shoot (high goal)
        scorer.gripperClose();
        scorer.flipper.setPosition(0.55);
        //drive.follow(traj1);
        for (int i = 0; i < 3; i++) {
            outtakeTimeout.reset();
            while ((scorer.outtake.getVelocity() < 2000 || scorer.outtake.getVelocity() > 2100) && outtakeTimeout.milliseconds() < 1000){
            }
            scorer.stopperClose();
            sleep(2000);
            scorer.stopperOpen();
            sleep(2000);
        }
        scorer.outtakeOff();
        scorer.stopperClose();
        //scorer.releaseOpen();
//
//        // Drop the wobble goal
//        drive.follow(traj2);
//        scorer.armOut();
//        scorer.gripperOpen();
//        sleep(400);
//        scorer.armIn(150);
//
//        // Go back to pick up the rings
//        scorer.intakeSlow();
//        scorer.outtakeOn(-370);
//        drive.follow(traj3);
//        drive.follow(traj3Z);
//        sleep(300);
//        outtakeTimeout.reset();
//        while ((Math.abs(scorer.outtake.getVelocity() - 1430) >= 30) && outtakeTimeout.milliseconds() < 1000) {
//        }
//        scorer.stopperClose();
//        sleep(200);
//        scorer.stopperOpen();
//        sleep(200);
//        scorer.stopperClose();
//        drive.follow(traj3a);
//        scorer.outtakeOff();
//        drive.follow(traj3b);
//        drive.follow(traj3c);
//
//        // Drive near wobble goal
//        // drive.turn(Math.toRadians(180));
//        drive.turn(Math.toRadians(-180));
//        scorer.intakeOff();
//        drive.follow(traj4);
//
//        // Lower wobble arm
//        scorer.gripperOpen();
//        scorer.armOut();
//
//        // Drive to wobble goal
//        drive.follow(traj5);
//
//        // Grab wobble goal
//        scorer.gripperClose();
//
//        // Drive to Zone
//        scorer.outtakeOn(50);
//        drive.follow(traj6);
//
//        // Dropping wobble goal
//        scorer.gripperOpen();
//        sleep(300);
//        // TODO: Make this a temporal marker
//        scorer.armIn(180 + (target == Target.A ? -180 : 0));
//
//        // Parking maneuver
//        if (target == Target.B) {
//            drive.follow(traj7);
//        } else if (target == Target.C) {
//            drive.follow(drive.from(traj6.end())
//                    .lineToLinearHeading(new Pose2d(-4, 31, Math.toRadians(-2)))
//                    .build());
//            for (int i = 0; i < 3; i++) {
//                scorer.stopperClose();
//                System.out.println(i);
//                outtakeTimeout.reset();
//                while ((scorer.outtake.getVelocity() < 1820 && outtakeTimeout.milliseconds() < 1000) || outtakeTimeout.milliseconds()<300) {
//                }
//                System.out.println(outtakeTimeout.milliseconds());
//                System.out.println(scorer.outtake.getVelocity());
//                System.out.println("BOBJOE");
//                scorer.stopperOpen();
//                sleep(300);
//                System.out.println(scorer.outtake.getVelocity());
//            }
//            scorer.stopperClose();
//            scorer.outtakeOff();
//            drive.follow(drive.from(new Pose2d(-4, 31, Math.toRadians(-3)))
//                    .forward(5)
//                    .build());
//        } else {
//            drive.follow(drive.from(traj6.end()).forward(3).build());
//        }
    }
}
