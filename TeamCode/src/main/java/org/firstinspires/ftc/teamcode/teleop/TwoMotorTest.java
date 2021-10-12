package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@TeleOp(name="Two Motor Test", group = "teleop")
public class TwoMotorTest extends LinearOpMode {
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

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        double now = 0;
        while (!isStopRequested()) {
            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();*/

            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, 0, kD, kF));
            //motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, 0, kD, kF));

            if(gamepad1.a) {
                motor1.setVelocity(targetVelo);
                motor2.setPower(motor1.getPower());
            } else if (gamepad1.b) {
                motor1.setVelocity(0);
                motor2.setPower(motor1.getPower());
            }

            if (gamepad1.left_bumper) {
                now = timer.milliseconds();
                servo1.setPosition(0.2);
            }

            if(now > 0 && timer.milliseconds() >= now + 100) {
                servo1.setPosition(0.06);
                now = 0;
            }


            telemetry.addData("Target", targetVelo);
            telemetry.addData("Motor1 Velo", motor1.getVelocity());

            telemetry.update();
        }
    }
}
