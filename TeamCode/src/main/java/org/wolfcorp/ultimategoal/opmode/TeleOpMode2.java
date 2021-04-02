package org.wolfcorp.ultimategoal.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@Config
@Disabled
@TeleOp(name="Testing TeleOp (Not CURRENT)", group = "drive")
public class TeleOpMode2 extends LinearOpMode {
    Drivetrain drive;
    Scorer scorer;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive = new Drivetrain(hardwareMap, true);
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

        scorer.openRelease();

        timer.reset();
        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            // Drivetrain
            double angle = -Drivetrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            drive.drive(-gamepad1.right_stick_y * Math.cos(angle) + gamepad1.right_stick_x * Math.sin(angle),
                    gamepad1.right_stick_y * Math.sin(angle) + gamepad1.right_stick_x * Math.cos(angle),
                    gamepad1.left_stick_x, 0.4, gamepad1.right_trigger > 0.8);
/*
            if(gamepad1.right_trigger > 0.8) {
                *//*double[] arr = drive.trueAimBot();
                double distance = arr[0];
                double degree = arr[1];
                packet.put("distance", distance);
                packet.put("deg", degree);
                dashboard.sendTelemetryPacket(packet);
                while (gamepad1.right_trigger > 0.8) {
                    //drive.resetAngle();
                    drive.turnTo(degree);
                }*//*

            }*/
            drive.turnForward(gamepad1.right_bumper);

            // Reversing scoring mechanisms
            scorer.reverse(gamepad1.b, 200);

            // Intake
            scorer.toggleIntake(gamepad1.a && !gamepad1.start, 1, 200);
            scorer.toggleIntakeRelease(gamepad1.dpad_right || gamepad2.right_bumper);

            // Outtake
            scorer.toggleStopper(gamepad1.dpad_left, gamepad1.x, 200);
            scorer.toggleOuttake(gamepad1.y || gamepad2.y, gamepad1.left_stick_button || gamepad2.a, 200);

            // Wobble Goal
            scorer.wobbleGripper(gamepad1.left_bumper || gamepad2.left_bumper, 200);
            scorer.wobbleArm(gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.dpad_up || gamepad2.dpad_up);
            scorer.moveFlipper(gamepad2.b && !gamepad2.start, gamepad2.x);

            // Powershot
            powershot(gamepad1.back);

            // LED vision signal
            if (timer.seconds() > 110) {
                // reminder for power shots
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
            }
            else if (timer.seconds() > 85) {
                // reminder for wobble goals / signal endgame
                scorer.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST);
            }

            // TODO: add button for manually resetting encoders after banging against wall
            //  (odometry gets more and more inaccurate over time)

            packet.put("vel", scorer.outtake.getVelocity());
            packet.put("shoot", scorer.stopper.getPosition() < 0.34 ? 2200 : 1500);
            packet.put("robot speed", drive.rightFront.getPower());
            packet.put("heading", Drivetrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

            dashboard.sendTelemetryPacket(packet);
            drive.update(); // odometry update
        }
    }

    public static double ALIGN_STRAFE = 5;
    public static double SIDESTEP_1 = 24;
    public static double SIDESTEP_2 = 4.5;
    public static double SIDESTEP_3 = 14;

    public void powershot(boolean condition) {

        if (condition && drive != null && scorer != null) {
            Trajectory align = drive.from(drive.getPoseEstimate(), 10, 10)
                    .strafeRight(ALIGN_STRAFE)
                    .build();
            Pose2d current = new Pose2d(0, -24 + 7, 0);
            drive.setPoseEstimate(current);
            Pose2d pose1 = new Pose2d(current.getX(), current.getY() + SIDESTEP_1, 0);
            Pose2d pose2 = new Pose2d(pose1.getX(), pose1.getY() + SIDESTEP_2, 0);
            Pose2d pose3 = new Pose2d(pose2.getX(), pose2.getY() + SIDESTEP_3, 0);
            Trajectory traj1 = drive.from(drive.getPoseEstimate())
                    //.strafeLeft(SIDESTEP_1)
                    .lineToLinearHeading(pose1)
                    .build();
            Trajectory traj2 = drive.from(traj1.end())
                    //.strafeLeft(SIDESTEP_2)
                    .lineToLinearHeading(pose2)
                    .build();
            Trajectory traj3 = drive.from(traj2.end())
                    //.strafeLeft(SIDESTEP_3)
                    .lineToLinearHeading(pose3)
                    .build();
            long sleepDuration = 340;
            scorer.outtakeOn(-100);
            sleep(700);
            drive.follow(traj1);
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
}
