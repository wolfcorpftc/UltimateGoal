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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.wolfcorp.ultimategoal.robot.Drivetrain;
import org.wolfcorp.ultimategoal.robot.Scorer;

@Config
@TeleOp(name="Current TeleOp", group = "drive")
public class TeleOpMode extends LinearOpMode {
    Drivetrain drive;
    Scorer scorer;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive = new Drivetrain(hardwareMap);
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
            drive.drive(-gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x, 0.4, gamepad1.right_bumper);
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
            drive.turnForward(gamepad1.right_trigger > 0.8);

            // Reversing scoring mechanisms
            scorer.reverse(gamepad1.b, 200);

            // Intake
            scorer.toggleIntake(gamepad1.a && !gamepad1.start, 1, 200);
            scorer.toggleIntakeFlipper(gamepad1.dpad_right || gamepad2.right_bumper);

            // Outtake
            scorer.toggleStopper(gamepad1.dpad_left, gamepad1.x, 200);
            scorer.toggleOuttake(gamepad1.y || gamepad2.y, gamepad1.left_stick_button || gamepad2.a, 200);

            // Wobble Goal
            scorer.wobbleGripper(gamepad1.left_bumper || gamepad2.left_bumper, 200);
            scorer.wobbleArm(gamepad1.dpad_down || gamepad2.dpad_down, gamepad1.dpad_up || gamepad2.dpad_up);

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
            packet.put("heading", drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

            dashboard.sendTelemetryPacket(packet);
            drive.update(); // odometry update
        }
    }

    public static double SIDESTEP_1 = 23;
    public static double SIDESTEP_2 = 15.5;
    public static double SIDESTEP_3 = 13.75;

    public void powershot(boolean condition) {
        if (condition && drive != null && scorer != null) {
            Trajectory traj1 = drive.from(drive.getPoseEstimate())
                    .strafeLeft(SIDESTEP_1)
                    .build();
            Trajectory traj2 = drive.from(traj1.end())
                    .strafeLeft(SIDESTEP_2)
                    .build();
            Trajectory traj3 = drive.from(traj2.end())
                    .strafeLeft(SIDESTEP_3)
                    .build();
            long sleepDuration = 340;
            scorer.outtakeOn();
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
