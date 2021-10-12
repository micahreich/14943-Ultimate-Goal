package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemIntake {
    DcMotor intake1, intake2;

    public SubsystemIntake(DcMotor _intake1, DcMotor _intake2) {
        intake1 = _intake1;
        intake2 = _intake2;

        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void activate(double power) {
        intake1.setPower(-power);
        intake2.setPower(power);
    }

    public void reset() {
        intake1.setPower(0.0);
        intake2.setPower(0.0);
    }
}
