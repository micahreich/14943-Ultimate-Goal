package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

public class SubsystemShooter {
    DcMotorEx shooter1, shooter2;
    DistanceSensor hopperHeightSensor;

    private final double TICKS_PER_SEC_THRESHOLD = 30;
    private final double DISTANCE_THRESHOLD = 0.6;

    public enum SHOOTER_STATE {
        HIGH_GOAL,
        POWER_SHOT,
        IDLE
    }

    public HashMap<SHOOTER_STATE, Double> shooterPowers = new HashMap<SHOOTER_STATE, Double>();

    public enum HOPPER_HEIGHT {
        ZERO,
        ONE,
        TWO,
        THREE,
        OVERFLOW
    }

    public HashMap<HOPPER_HEIGHT, Double> hopperHeights = new HashMap<HOPPER_HEIGHT, Double>();

    public SubsystemShooter(DcMotorEx _shooter1, DcMotorEx _shooter2, DistanceSensor _hopperHeightSensor) {
        shooter1 = _shooter1;
        shooter2 = _shooter2;
        hopperHeightSensor = _hopperHeightSensor;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(150, 0, 30, 12));

        shooterPowers.put(SHOOTER_STATE.HIGH_GOAL, (double) 1780);
        shooterPowers.put(SHOOTER_STATE.POWER_SHOT, (double) 1350);
        shooterPowers.put(SHOOTER_STATE.IDLE, (double) 0);

        hopperHeights.put(HOPPER_HEIGHT.ZERO, (double) 13.1);
        hopperHeights.put(HOPPER_HEIGHT.ONE, (double) 12.2);
        hopperHeights.put(HOPPER_HEIGHT.TWO, (double) 11.1);
        hopperHeights.put(HOPPER_HEIGHT.THREE, (double) 9.1);
    }

    public HOPPER_HEIGHT getHopperHeight() {
        double distance = hopperHeightSensor.getDistance(DistanceUnit.CM);

        for(Map.Entry entry : hopperHeights.entrySet()) {
            if(Math.abs(distance - (double) entry.getValue()) <= DISTANCE_THRESHOLD) {
                return (HOPPER_HEIGHT) entry.getKey();
            }
        }

        return HOPPER_HEIGHT.OVERFLOW;
    }

    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
        /*shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));*/
    }

    public boolean isReady(double targetVelocity) {
        return Math.abs(targetVelocity - shooter1.getVelocity()) <= 30;
    }

    public void activate(SHOOTER_STATE shooterState) {
        shooter1.setVelocity(shooterPowers.get(shooterState));
        shooter2.setPower(shooter1.getPower());
    }

    public void activate(int targetVelo) {
        shooter1.setVelocity(targetVelo);
        shooter2.setPower(shooter1.getPower());
    }

    public void reset() {
        shooter1.setVelocity(shooterPowers.get(SHOOTER_STATE.IDLE));
        shooter2.setPower(shooter1.getPower());
    }
}
