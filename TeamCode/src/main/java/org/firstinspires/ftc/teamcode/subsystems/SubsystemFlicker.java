package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SubsystemFlicker {
    Servo flicker;

    private final double SERVO_TIMEOUT = 110;
    private final double MAX_POSITION = 0.2;
    private final double MIN_POSITION = 0.05;

    ElapsedTime timer;

    public SubsystemFlicker(Servo _flicker, ElapsedTime _timer) {
        flicker = _flicker;
        flicker.setDirection(Servo.Direction.FORWARD);

        timer = _timer;
    }

    public void activate() {
        double startTime = timer.milliseconds();

        flicker.setPosition(MAX_POSITION);
        while(Math.abs(timer.milliseconds() - startTime) <= SERVO_TIMEOUT) {
            // do nothing
        }
        flicker.setPosition(MIN_POSITION);
        /*while(Math.abs(timer.milliseconds() - startTime) <= SERVO_TIMEOUT) {
            // do nothing
        }*/
    }

    public void reset() {
        flicker.setPosition(MIN_POSITION);
    }
}
