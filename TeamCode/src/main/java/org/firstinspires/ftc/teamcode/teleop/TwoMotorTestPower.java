package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(name="Two Motor Test Power", group = "teleop")
public class TwoMotorTestPower extends LinearOpMode {
    DcMotorEx motor1, motor2;
    Servo servo1;

    public static double kP = 1400;
    public static double kD = 320;
    public static double kF = 36;

    public static double targetVelo = 600;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        servo1 = hardwareMap.get(Servo.class, "flicker");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(kP, 0, kD, kF);

        /*motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);*/

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        double now = 0;
        while (!isStopRequested()) {
            //motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, 0, kD, kF));
            //motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, 0, kD, kF));

            if(gamepad1.a) {
                motor1.setPower(1.0);
                motor2.setPower(1.0);
            } else if (gamepad1.b) {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            telemetry.addData("Motor1 Velo", motor1.getVelocity());

            telemetry.update();
        }
    }
}
