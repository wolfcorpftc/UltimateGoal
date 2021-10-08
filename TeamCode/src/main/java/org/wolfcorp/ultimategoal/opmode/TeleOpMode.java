package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@Config
@TeleOp(name="Current TeleOp", group = "drive")
public class TeleOpMode extends LinearOpMode {
    Drivetrain drive;
    Scorer scorer;

    private ElapsedTime powershotTurnDelay = new ElapsedTime();
    private ElapsedTime wallBangDelay = new ElapsedTime();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive = new Drivetrain(hardwareMap, false);
        drive.setTelemetry(false);
        scorer = new Scorer(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        Pose2d initialPose = new Pose2d(12, 36, Math.toRadians(0));

        drive.setPoseEstimate(initialPose);

        waitForStart();

        scorer.outtakeDelay.reset();
        scorer.releaseOpen();
        scorer.stopperClose();

        timer.reset();
        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            // Drivetrain
            drive.drive(Math.abs(gamepad1.right_stick_y) < 0.8 && Math.abs(gamepad1.right_stick_y) > 0.05 ? gamepad1.right_stick_y > 0 ? -0.5 : 0.5 : -gamepad1.right_stick_y,
                    Math.abs(gamepad1.right_stick_x) < 0.8 && Math.abs(gamepad1.right_stick_x) > 0.05 ? (gamepad1.right_stick_x > 0 ? 0.5 : -0.5) : gamepad1.right_stick_x,
                    Math.abs(gamepad1.left_stick_x) < 0.6 && Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x > 0 ? 0.3 : -0.3 : gamepad1.left_stick_x, 0.4, gamepad1.right_trigger > 0.8);
//
//            if(gamepad1.right_trigger > 0.8) {
//                double[] arr = drive.aim();
//                double distance = arr[0];
//                double degree = arr[1];
//                while (gamepad1.right_trigger > 0.8) {
//                    //drive.resetAngle();
//                    drive.turnTo(degree);
//                }
//            }
//            if (gamepad1.right_bumper) {
//                drive.aim();
//            }

            drive.turnToForward(gamepad1.right_bumper);
            drive.resetIMU(gamepad1.left_trigger > 0.8);

            // Reversing scoring mechanisms
            scorer.reverse(gamepad1.b, 200);

            // Intake
            scorer.toggleIntake(gamepad1.a && !gamepad1.start, 1, 500);
            if(gamepad1.a && !gamepad1.start){
                scorer.resetIntakeDipFlag();
            }
            scorer.toggleIntakeRelease(gamepad1.dpad_right || gamepad2.right_bumper);

            // Outtake
            scorer.toggleStopper(gamepad1.dpad_left, gamepad1.x, 500);
            scorer.toggleOuttake(gamepad1.y || gamepad2.y, gamepad1.left_stick_button || (gamepad2.a && !gamepad2.start), 200);
            scorer.toggleUnsticker(gamepad2.left_stick_button);

            // Wobble Goal
            scorer.wobbleGripper(gamepad1.left_bumper || gamepad2.left_bumper, 200);
            scorer.wobbleArm(gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.dpad_up || gamepad2.dpad_up);
            scorer.moveFlipper(gamepad2.b && !gamepad2.start, gamepad2.x);
            // Powershot
            powershot(gamepad1.back);

            // LED vision signal
            if (timer.seconds() > 110) {
                // reminder for wobble goals / signal endgame
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST);
            } else if (timer.seconds() > 85) {
                // reminder for wobble goals / signal endgame
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST);
            }

            // TODO: add button for manually resetting encoders after banging against wall
            //  (odometry gets more and more inaccurate over time)
            //packet.put("outtake shooting", scorer.stopper.getPosition() < 0.34 ? 2200 : 1500);
/*
            packet.put("robot speed", drive.rightFront.getPower());
            packet.put("heading", drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
*/

            // Detect rings from intake
            scorer.updateRingCount();
            packet.put("intake rings", scorer.intakeRings);
            packet.put("intake vel", -  scorer.intake.getVelocity());
            packet.put("outtake", scorer.outtake.getVelocity());
////            packet.put("LB vel", drive.leftBack.getVelocity());
////            packet.put("LF vel", drive.leftFront.getVelocity());
////            packet.put("RB vel", drive.rightBack.getVelocity());
////            packet.put("RF vel", drive.rightFront.getVelocity());
//            dashboard.sendTelemetryPacket(packet);
            drive.update(); // odometry update
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void autoshoot(boolean condition) {
        if (condition && wallBangDelay.milliseconds() > 350) {
            wallBangDelay.reset();
            int duration = 400;
            drive.turnTo(0);
            scorer.flipper.setPosition(.33);
            scorer.outtakeOn(200);
            scorer.stopperClose();
            sleep(duration);
            scorer.stopperOpen();
            sleep(duration);
            scorer.stopperClose();
            sleep(duration);
            scorer.stopperOpen();
            sleep(duration);
            scorer.stopperClose();
            sleep(duration);
            scorer.stopperOpen();
            sleep(duration);
            scorer.stopperClose();
            scorer.flipper.setPosition(.7);
        }
    }

    public static double ALIGN_STRAFE = 2;
    public static double SIDESTEP_1 = 23.5;
    public static double SIDESTEP_2 = 8;
    public static double SIDESTEP_3 = 9;

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
            Trajectory traj1 = drive.from(wall, 15, 12)
                    //.strafeLeft(SIDESTEP_1)
                    .lineToLinearHeading(pose1)
                    .build();
            Trajectory traj2 = drive.from(traj1.end(), 15, 12)
                    //.strafeLeft(SIDESTEP_2)
                    .lineToLinearHeading(pose2)
                    .build();
            Trajectory traj3 = drive.from(traj2.end(), 15, 12)
                    //.strafeLeft(SIDESTEP_3)
                    .lineToLinearHeading(pose3)
                    .build();
            long sleepDuration = 200;
            scorer.outtakeOn(-600);
            drive.follow(align);
            //drive.sidestepRight(0.5, ALIGN_STRAFE);
            drive.setPoseEstimate(wall);
            sleep(sleepDuration);
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
            long sleepDuration = 500;
            scorer.outtakeOn(-700);
            Trajectory align = drive.from(drive.getPoseEstimate())
                    .strafeRight(5)
                    .build();
            drive.follow(align);
            sleep(sleepDuration*2);
            drive.turnTo(14);
            scorer.stopperClose();
            sleep(sleepDuration);
            scorer.stopperOpen();
            sleep(sleepDuration);
            drive.turnTo(4);
            scorer.stopperClose();
            sleep(sleepDuration);
            scorer.stopperOpen();
            sleep(sleepDuration);
            drive.turnTo(6.25);
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
