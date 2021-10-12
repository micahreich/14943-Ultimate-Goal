package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

public class SubsystemWobbleArm {
    Servo wobble1, wobble2, wobbleClaw;

    private final double SERVO_TIMEOUT = 200;

    public enum ARM_STATE {
        RETRACTED,
        LOWER,
        UPRIGHT
    }

    public HashMap<ARM_STATE, Double> armStates = new HashMap<ARM_STATE, Double>();

    public enum CLAW_STATE {
        OPEN,
        CLOSED
    }

    public HashMap<CLAW_STATE, Double> clawStates = new HashMap<CLAW_STATE, Double>();

    ElapsedTime timer;

    public SubsystemWobbleArm(Servo _wobble1, Servo _wobble2, Servo _wobbleClaw, ElapsedTime _timer) {
        wobble1 = _wobble1;
        wobble2 = _wobble2;
        wobbleClaw = _wobbleClaw;

        wobble1.setDirection(Servo.Direction.FORWARD);
        wobble2.setDirection(Servo.Direction.FORWARD);
        wobbleClaw.setDirection(Servo.Direction.FORWARD);

        armStates.put(ARM_STATE.RETRACTED, (double) 1.0);
        armStates.put(ARM_STATE.LOWER, (double) 0.15);
        armStates.put(ARM_STATE.UPRIGHT, (double) 0.4);

        clawStates.put(CLAW_STATE.OPEN, (double) 0.0);
        clawStates.put(CLAW_STATE.CLOSED, (double) 0.4);

        timer = _timer;
    }

    public void activate(ARM_STATE armState, CLAW_STATE clawState, double delay, boolean clawFirst) {
        double startTime = timer.milliseconds();

        if (clawFirst) {
            wobbleClaw.setPosition(clawStates.get(clawState));
            while(Math.abs(timer.milliseconds() - startTime) <= delay) {
                // do nothing
            }

            wobble1.setPosition(armStates.get(armState));
            wobble2.setPosition(armStates.get(armState));
        } else {
            wobble1.setPosition(armStates.get(armState));
            wobble2.setPosition(armStates.get(armState));
            while(Math.abs(timer.milliseconds() - startTime) <= delay) {
                // do nothing
            }

            wobbleClaw.setPosition(clawStates.get(clawState));
        }
    }

    public void activate(double position, CLAW_STATE clawState, double delay, boolean clawFirst) {
        double startTime = timer.milliseconds();

        if (clawFirst) {
            wobbleClaw.setPosition(clawStates.get(clawState));
            while(Math.abs(timer.milliseconds() - startTime) <= delay) {
                // do nothing
            }

            wobble1.setPosition(position);
            wobble2.setPosition(position);
        } else {
            wobble1.setPosition(position);
            wobble2.setPosition(position);
            while(Math.abs(timer.milliseconds() - startTime) <= delay) {
                // do nothing
            }

            wobbleClaw.setPosition(clawStates.get(clawState));
        }
    }

    public void activate(ARM_STATE armState) {
        wobble1.setPosition(armStates.get(armState));
        wobble2.setPosition(armStates.get(armState));
    }

    public void activate(double position) {
        wobble1.setPosition(position);
        wobble2.setPosition(position);
    }

    public void activateClaw(double position) {
        wobbleClaw.setPosition(position);
    }

    public void activateClaw(CLAW_STATE clawState) {
        wobbleClaw.setPosition(clawStates.get(clawState));
    }

    public void reset() {
        wobble1.setPosition(armStates.get(ARM_STATE.RETRACTED));
        wobble2.setPosition(armStates.get(ARM_STATE.RETRACTED));
        wobbleClaw.setPosition(clawStates.get(CLAW_STATE.OPEN));
    }

    public boolean checkClawPosition(double target) {
        return Math.abs(wobbleClaw.getPosition() - target) < 0.1;
    }

    public boolean checkArmPosition(double target) {
        return Math.abs(wobble1.getPosition() - target) < 0.1;
    }

}
