package org.wolfcorp.ultimategoal.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Scorer {
    public DcMotorEx intake, outtake, arm;
    public Servo gripper, stopper;

    public boolean reverse = false;
    public boolean pastDirectionUp = false;

    private ElapsedTime intakeDelay = new ElapsedTime();
    private ElapsedTime outtakeDelay = new ElapsedTime();
    private ElapsedTime gripperDelay = new ElapsedTime();
    private ElapsedTime reverseDelay = new ElapsedTime();

    public Scorer(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        gripper = hardwareMap.get(Servo.class, "gripper");
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    public void intakeToggle(boolean condition, double speed, int toggleDelay) {
        if (condition && intakeDelay.milliseconds() > toggleDelay) {
            intake.setPower(intake.getPower() == 0 ? speed * (reverse ? 1 : -1) : 0);
            intakeDelay.reset();
        }
    }

    //TODO: Have to wait a bit after setting power, then also have to move stopper to correct position
    public void outtakeToggle(boolean condition, double speed, int toggleDelay) {
        if (condition && outtakeDelay.milliseconds() > toggleDelay) {
            outtake.setPower(outtake.getPower() == 0 ? speed * (reverse ? -1 : 1) : 0);
            outtakeDelay.reset();
        }
    }

    public void wobbleGripper(boolean condition, int toggleDelay) {
        if (condition && gripperDelay.milliseconds() > toggleDelay) {
            gripper.setPosition(gripper.getPosition() == 0.42 ? 0.8 : 0.42);
            gripperDelay.reset();
        }
    }

    public void wobbleArm(boolean condition) {
        if (condition) {
            arm.setPower(0.5 * (reverse ? -1 : 1));
        } else {
            arm.setPower(0);
        }
    }

    public void reverse(boolean condition, int toggleDelay) {
        if (condition && reverseDelay.milliseconds() > toggleDelay) {
            reverse = !reverse;
            intake.setPower(intake.getPower() != 0 ? intake.getPower() * (reverse ? 1 : -1) : 0);
            outtake.setPower(outtake.getPower() != 0 ? outtake.getPower() * (reverse ? 1 : -1) : 0);
            reverseDelay.reset();
        }
    }
}
