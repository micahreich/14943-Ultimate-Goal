package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemCamera;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name="Vision OpMode Test", group = "teleop")
public class VisionOpModeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SubsystemCamera vision = new SubsystemCamera(this, hardwareMap, telemetry);
        vision.initialize();

        while(!isStarted()) {
            telemetry.addData("Height", vision.getHeight());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // none
        }
    }
}
