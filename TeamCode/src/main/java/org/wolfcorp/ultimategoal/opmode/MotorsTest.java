package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@Config
@TeleOp(name="Motor Testing", group = "drive")
public class MotorsTest extends LinearOpMode {
    Drivetrain drive;
    Scorer scorer;

    private ElapsedTime powershotTurnDelay = new ElapsedTime();

    @Override
    public void runOpMode() {
        drive = new Drivetrain(hardwareMap, false);
        drive.setTelemetry(false);
        waitForStart();

        telemetry.addData("Buttons", "a=LF, b=LB, x=RF, y=RB");
        telemetry.update();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                drive.leftFront.setPower(0.5);
            } else if (gamepad1.b) {
                drive.leftBack.setPower(0.5);
            } else if (gamepad1.x) {
                drive.rightFront.setPower(0.5);
            } else if (gamepad1.y) {
                drive.rightBack.setPower(0.5);
            }
        }
    }

    public static double ALIGN_STRAFE = 2;
    public static double SIDESTEP_1 = 25;
    public static double SIDESTEP_2 = 9;
    public static double SIDESTEP_3 = 11;

    public void powershot(boolean condition) {

        if (condition && drive != null && scorer != null) {
            Trajectory align = drive.from(drive.getPoseEstimate(), 25, 25)
                    .strafeRight(ALIGN_STRAFE)
                    .build();
            Pose2d wall = new Pose2d(0, -24 + 7, 0);
            //drive.setPoseEstimate(current);
            Pose2d pose1 = new Pose2d(wall.getX(), wall.getY() + SIDESTEP_1, 0);
            Pose2d pose2 = new Pose2d(wall.getX(), wall.getY() + SIDESTEP_2 + SIDESTEP_1, 0);
            Pose2d pose3 = new Pose2d(wall.getX(), wall.getY() + SIDESTEP_3 + SIDESTEP_2 + SIDESTEP_1, 0);
            Trajectory traj1 = drive.from(wall, 25, 25)
                    //.strafeLeft(SIDESTEP_1)
                    .lineToLinearHeading(pose1)
                    .build();
            Trajectory traj2 = drive.from(traj1.end(), 25, 25)
                    //.strafeLeft(SIDESTEP_2)
                    .lineToLinearHeading(pose2)
                    .build();
            Trajectory traj3 = drive.from(traj2.end(), 25, 25)
                    //.strafeLeft(SIDESTEP_3)
                    .lineToLinearHeading(pose3)
                    .build();
            long sleepDuration = 350;
            scorer.outtakeOn(-250);
            drive.follow(align);
            //drive.sidestepRight(0.5, ALIGN_STRAFE);
            drive.setPoseEstimate(wall);
            sleep(sleepDuration*3/2);
            drive.follow(traj1);
            sleep(sleepDuration);
            scorer.stopperOpen();
            sleep(sleepDuration);
            scorer.stopperClose();
            drive.follow(traj2);
            sleep(sleepDuration);
            scorer.stopperOpen();
            sleep(sleepDuration);
            scorer.stopperClose();
            drive.follow(traj3);
            sleep(sleepDuration);
            scorer.stopperOpen();
            sleep(sleepDuration);
            scorer.stopperClose();
            scorer.outtakeOff();
        }
    }

    public void powershotTurn(boolean condition) {
        if (condition) {
            long sleepDuration = 400;
            scorer.outtakeOn(-300);
            sleep(sleepDuration);
            drive.turn(Math.toRadians(12));
            scorer.stopperClose();
            sleep(sleepDuration);
            scorer.stopperOpen();
            drive.turn(Math.toRadians(3));
            scorer.stopperClose();
            sleep(sleepDuration);
            scorer.stopperOpen();
            drive.turn(Math.toRadians(3));
            scorer.stopperClose();
            sleep(sleepDuration);
            scorer.stopperOpen();
            sleep(sleepDuration);
            scorer.stopperClose();
            scorer.outtakeOff();
        }
//        if (condition && powershotTurnDelay.milliseconds() > 500) {
//            powershotTurnDelay.reset();
//            drive.turn(Math.toRadians(-5));
//        }
    }
}
