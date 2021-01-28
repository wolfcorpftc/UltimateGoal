package org.wolfcorp.ultimategoal.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

public class Scorer {
    public DcMotorEx intake, outtake, arm;
    public Servo gripper, stopper;

    public boolean reverse = false;

    private ElapsedTime intakeDelay = new ElapsedTime();
    private ElapsedTime outtakeDelay = new ElapsedTime();
    private ElapsedTime stopperDelay = new ElapsedTime();
    private ElapsedTime gripperDelay = new ElapsedTime();
    private ElapsedTime reverseDelay = new ElapsedTime();
    private ElapsedTime autoArm = new ElapsedTime();

    public Scorer(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    //TODO: Have to wait a bit after setting power, then also have to move stopper to correct position
    public void toggleOuttake(boolean condition, int toggleDelay) {
        if (condition && outtakeDelay.milliseconds() > toggleDelay) {
            outtake.setVelocity(outtake.getVelocity() == 0 ? 1680 : 0);
            stopper.setPosition(0.34);
            outtakeDelay.reset();
        }
    }

    public void toggleStopper(boolean manualCondition, boolean automaticCondition, int toggleDelay) {
        if ((manualCondition || automaticCondition) && stopperDelay.milliseconds() > toggleDelay) {
            int amount = manualCondition ? 1 : 3;
            for (int i = 0; i < amount; i++) {
                stopper.setPosition(0.02);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                stopper.setPosition(0.34);
                try {
                    sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            stopperDelay.reset();
        }
    }

    public void wobbleGripper(boolean condition, int toggleDelay) {
        if (condition && gripperDelay.milliseconds() > toggleDelay) {
            gripper.setPosition(gripper.getPosition() == 0.42 ? 0.8 : 0.42);
            gripperDelay.reset();
        }
    }

    public void wobbleArm(boolean conditionUp, boolean conditionDown) {
        if (conditionUp) {
            arm.setPower(0.5);
        } else if (conditionDown) {
            arm.setPower(-0.5);
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
        gripper.setPosition(0.42);
        while(autoArm.milliseconds()<1000){

        }
    }

    public void gripperClose(){
        gripper.setPosition(0.8);
        while(autoArm.milliseconds()<1000){

        }
    }

    public void armDown(){
        autoArm.reset();
        arm.setPower(-0.5);
        while(autoArm.milliseconds()<2000){

        }
        arm.setPower(0);
    }

    public void armUp(){
        autoArm.reset();
        arm.setPower(0.5);
        while(autoArm.milliseconds()<2000){

        }
        arm.setPower(0);
    }

    public void intake(int millisDuration) {
        intake.setPower(1);
        try { sleep(millisDuration); } catch(Exception e) {}
        intake.setPower(0);
    }

    public void outtakeOn() {
        outtake.setPower(1);
    }

    public void outtakeOff() {
        outtake.setPower(0);
    }

    public void stopperOpen() {
        stopper.setPosition(0.02);
    }

    public void stopperClose() {
        stopper.setPosition(0.34);
    }
}
