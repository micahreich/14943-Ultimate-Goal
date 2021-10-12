package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="Servo Zero Test", group = "teleop")
public class ServoZeroTest extends LinearOpMode {
    Servo servo1, servo2, servo3;

    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servo1 = hardwareMap.get(Servo.class, "wobble1");
        servo2 = hardwareMap.get(Servo.class, "wobble2");
        servo3 = hardwareMap.get(Servo.class, "wobbleClaw");

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3.setDirection(Servo.Direction.FORWARD);


        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

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

            if (gamepad1.a) {
                servo1.setPosition(0.9);
                servo2.setPosition(0.9);
            } else if (gamepad1.b) {
                servo1.setPosition(0.1);
                servo2.setPosition(0.1);
            } else if(gamepad1.x) {
                servo1.setDirection(Servo.Direction.REVERSE);
            } else if (gamepad1.dpad_left) {
                servo3.setPosition(0.0);
            } else if(gamepad1.dpad_right) {
                servo3.setPosition(0.4);
            }
        }
    }
}
