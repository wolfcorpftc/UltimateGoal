package org.wolfcorp.ultimategoal.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

@Config
public class Scorer {
    public DcMotorEx intake, outtake, arm;
    public Servo gripper, stopper, release, release2;
    public RevBlinkinLedDriver LED;

    public boolean reverse = false;
    public int fireAmount = 0;

    public static PIDFCoefficients outtakeCoeff = new PIDFCoefficients(45, 0.16, 0, 16);

    private boolean xclick = true;

    private ElapsedTime intakeDelay = new ElapsedTime();
    private ElapsedTime flipperDelay = new ElapsedTime();
    private ElapsedTime outtakeDelay = new ElapsedTime();
    private ElapsedTime stopperDelay = new ElapsedTime();
    private ElapsedTime fireDelay = new ElapsedTime();
    private ElapsedTime gripperDelay = new ElapsedTime();
    private ElapsedTime reverseDelay = new ElapsedTime();

    public Scorer(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtakeCoeff);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        release = hardwareMap.get(Servo.class, "release");
        release2 = hardwareMap.get(Servo.class, "release2");
        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        gripper = hardwareMap.get(Servo.class, "gripper");
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    public void toggleIntake(boolean condition, double speed, int toggleDelay) {
        if (condition && intakeDelay.milliseconds() > toggleDelay) {
            intake.setPower(intake.getPower() == 0 ? speed * (reverse ? 1 : -1) : 0);
            intakeDelay.reset();
        } else {
            intake.setPower(intake.getPower() != 0 ? speed * (reverse ? 1 : -1) : 0);
        }
    }

    public void toggleIntakeFlipper(boolean condition) {
        if (condition && flipperDelay.milliseconds() > 200) {
            release2.setPosition(release2.getPosition() == 1 ? 0 : 1);
            flipperDelay.reset();
        }
    }

    public void toggleOuttake(boolean condition, boolean lowerMode, int toggleDelay) {
        if ((condition || lowerMode) && outtakeDelay.milliseconds() > toggleDelay) {
            outtake.setVelocity(outtake.getVelocity() == 0 ? ((lowerMode ? 0.65 : 1) * 2000) : 0);
            stopperClose();
            outtakeDelay.reset();
        }
    }

    public void toggleStopper(boolean manualCondition, boolean automaticCondition, int toggleDelay) {
        if ((manualCondition || automaticCondition) && xclick && stopperDelay.milliseconds() > toggleDelay) {
            xclick = false;
            fireAmount = manualCondition ? 1 : 3;
            fireDelay.reset();
        }
        if(!automaticCondition)
            xclick = true;
        if (fireAmount > 0) {
            if (fireDelay.milliseconds() > 680) {
                fireDelay.reset();
                fireAmount--;
            } else if (fireDelay.milliseconds() > 340) {
                stopperClose();
            } else {
                stopperOpen();
            }
        }
    }

    public void wobbleGripper(boolean condition, int toggleDelay) {
        if (condition && gripperDelay.milliseconds() > toggleDelay) {
            gripper.setPosition(gripper.getPosition() == 0.3 ? 1 : 0.3);
            gripperDelay.reset();
        }
    }

    public void wobbleArm(boolean conditionUp, boolean conditionDown) {
        if (conditionUp) {
            arm.setPower(0.8);
        } else if (conditionDown) {
            arm.setPower(-0.8);
        } else {
            arm.setPower(0);
        }
    }

    public void reverse(boolean condition, int toggleDelay) {
        if (condition && reverseDelay.milliseconds() > toggleDelay) {
            reverse = !reverse;
            reverseDelay.reset();
        }
    }

    public void gripperOpen(){
        gripper.setPosition(1);
        pause(400);
    }

    public void gripperClose(){
        gripper.setPosition(0.3);
        pause(600);
    }

    public void armOut(){
        arm.setPower(1);
        pause(570);
        arm.setPower(0);
    }

    public void armIn(){
        arm.setPower(-1);
        pause(1000);
        arm.setPower(0);
    }

    public void intake(int millisDuration) {
        intake.setPower(-1);
        pause(millisDuration);
        intake.setPower(0);
    }

    public void intakeOn() {
        intake.setPower(-1);
    }

    public void intakeSlow() {
        intake.setPower(-1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void outtakeOn() {
        outtake.setVelocity(1580);
    }

    public void outtakeOn(int difference) {
        outtake.setVelocity(1580+difference);
    }

    public void outtakeOff() {
        outtake.setVelocity(0);
    }

    public void stopperOpen() {
        stopper.setPosition(0.02);
    }

    public void stopperClose() {
        stopper.setPosition(0.34);
    }

    public void openRelease() {
        release.setPosition(1);
    }

    public void pause(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
