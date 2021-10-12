package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemFlicker;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemShooter;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemWobbleArm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// Main teleop program

@TeleOp(name="TeleOp Main", group = "teleop")
public class TeleOpMain extends LinearOpMode {
    // hardware objects
    DcMotor intake1, intake2;
    DcMotorEx shooter1, shooter2;
    Servo flicker1, wobble1, wobble2, wobbleClaw;
    DistanceSensor hopperHeightSensor;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double initialAngle = Math.PI;
    double globalAngle = 0;

    // subsystem objects
    SampleMecanumDrive drive;
    SubsystemIntake intake;
    SubsystemShooter shooter;
    SubsystemWobbleArm wobbleArm;
    SubsystemFlicker flicker;

    enum DriveMode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    DriveMode currentMode = DriveMode.DRIVER_CONTROL;
    Vector2d towerPosition = new Vector2d(72,36);

    boolean useIMU = true;

    int shooterCounterA = 1;
    int shooterCounterB = 1;
    int psCounter = 1;

    boolean shooterLastStateA = false;
    boolean shooterLastStateB = false;
    boolean psLastState = false;

    Trajectory toRightShot;
    Trajectory toMidShot;
    Trajectory toLeftShot;

    int shotCounter = 0;

    boolean armGoingUp = false;
    boolean armGoingDown = false;
    boolean armGoingMid = false;

    double triggerTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-63, 24.5, Math.toRadians(180));

        // counters, toggles
        SubsystemShooter.SHOOTER_STATE currentState = SubsystemShooter.SHOOTER_STATE.HIGH_GOAL;

        // timers
        ElapsedTime timer = new ElapsedTime();

        // hardware references
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(150, 0, 30, 12));

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        flicker1 = hardwareMap.get(Servo.class, "flicker");
        wobble1 = hardwareMap.get(Servo.class, "wobble1");
        wobble2 = hardwareMap.get(Servo.class, "wobble2");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");

        hopperHeightSensor = hardwareMap.get(DistanceSensor.class, "dist");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (Exception e) {
            useIMU = false;
        }

        // subsystem creation
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intake = new SubsystemIntake(intake1, intake2);
        shooter = new SubsystemShooter(shooter1, shooter2, hopperHeightSensor);
        wobbleArm = new SubsystemWobbleArm(wobble1, wobble2, wobbleClaw, timer);
        flicker = new SubsystemFlicker(flicker1, timer);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            // update pose
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();

            // controls depend on driver mode:
            switch(currentMode) {
                case DRIVER_CONTROL:
                    // drivetrain
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -0.95 * gamepad1.right_stick_x
                            )
                    );

                    // automatic aiming
                    if(gamepad1.a) {
                        double updatedHeading = getIMUHeading();
                        poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), updatedHeading);

                        Vector2d difference = towerPosition.minus(poseEstimate.vec());
                        double targetAngle = difference.angle();

                        // turn towards high goal
                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading() + Math.PI));

                        currentMode = DriveMode.AUTOMATIC_CONTROL;
                    }

                    if (gamepad1.y) {
                        drive.setPoseEstimate(new Pose2d(63, -15, Math.toRadians(180)));;
                        globalAngle = Math.PI;
                    }

                    if(gamepad1.b && !psLastState) {
                        psCounter++;
                    }
                    psLastState = gamepad1.b;

                    if (psCounter == 2) {
                        // powershot sequence initial
                        shooterCounterB++;
                        //flicker.activate();

                        toRightShot = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(-9, 4, Math.toRadians(180)))
                                .build();

                        drive.followTrajectoryAsync(toRightShot);
                        currentMode = DriveMode.AUTOMATIC_CONTROL;

                        psCounter++;

                    } else if (psCounter == 4) {
                        //flicker.activate();
                        toMidShot = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(-9, 8, Math.toRadians(180)))
                                .build();

                        drive.followTrajectoryAsync(toMidShot);
                        currentMode = DriveMode.AUTOMATIC_CONTROL;

                        psCounter++;
                    } else if (psCounter == 6) {
                        //flicker.activate();
                        toLeftShot = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(-9, 13, Math.toRadians(180)))
                                .build();

                        drive.followTrajectoryAsync(toLeftShot);
                        currentMode = DriveMode.AUTOMATIC_CONTROL;

                        psCounter++;
                    }

                    // shooter
                    if (gamepad2.a && !shooterLastStateA) {
                        shooterCounterA++;
                        shooterCounterB = 1;
                    }
                    if(gamepad2.b && !shooterLastStateB) {
                        shooterCounterB++;
                        shooterCounterA = 1;
                    }
                    shooterLastStateA = gamepad2.a;
                    shooterLastStateB = gamepad2.b;

                    if (shooterCounterA % 2 == 0) {
                        shooter.activate(SubsystemShooter.SHOOTER_STATE.HIGH_GOAL);
                        currentState = SubsystemShooter.SHOOTER_STATE.HIGH_GOAL;
                        //shooterCounterB = 1;
                    } else if(shooterCounterB % 2 == 0) {
                        shooter.activate(SubsystemShooter.SHOOTER_STATE.POWER_SHOT);
                        currentState = SubsystemShooter.SHOOTER_STATE.POWER_SHOT;
                        //shooterCounterA = 1;
                    } /*else if(shooterCounterA % 2 == 0 && shooterCounterB % 2 == 0 && currentState == SubsystemShooter.SHOOTER_STATE.HIGH_GOAL) {
                        shooter.activate(SubsystemShooter.SHOOTER_STATE.POWER_SHOT);
                        currentState = SubsystemShooter.SHOOTER_STATE.POWER_SHOT;
                        shooterCounterB++;
                    } else if(shooterCounterA % 2 == 0 && shooterCounterB % 2 == 0 && currentState == SubsystemShooter.SHOOTER_STATE.POWER_SHOT) {
                        shooter.activate(SubsystemShooter.SHOOTER_STATE.HIGH_GOAL);
                        currentState = SubsystemShooter.SHOOTER_STATE.HIGH_GOAL;
                        shooterCounterA++;
                    }*/ else {
                        shooter.reset();
                    }

                    // flicker
                    if (gamepad2.left_bumper) {
                        shotCounter++;
                        flicker.activate();
                    } else if(gamepad2.right_bumper && shotCounter == 0) {
                        shotCounter++;
                        shootRings(4);
                    } else if(gamepad2.right_bumper) {
                        shotCounter++;
                        shootRings(3);
                    }

                    // intake
                    if (gamepad2.y) {
                        intake.activate(1.0);
                    } else if (gamepad2.x) {
                        intake.activate(-1.0);
                    } else {
                        intake.reset();
                    }

                    // wobble arm
                    if (gamepad2.dpad_up) {
                        /*wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.RETRACTED,
                                SubsystemWobbleArm.CLAW_STATE.CLOSED,
                                500, true);*/

                        wobbleArm.activateClaw(0.4);
                        armGoingUp = true;
                        triggerTime = timer.milliseconds();
                    } else if(gamepad2.dpad_down) {
                        /*wobbleArm.activate(0.1,
                                SubsystemWobbleArm.CLAW_STATE.OPEN,
                                500, false);*/

                        wobbleArm.activate(0.1);
                        armGoingDown = true;
                        triggerTime = timer.milliseconds();
                    } else if(gamepad2.dpad_left) {
                        /*wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.UPRIGHT,
                                SubsystemWobbleArm.CLAW_STATE.OPEN,
                                500, false);*/

                        wobbleArm.activate(0.4);
                        armGoingMid = true;
                        triggerTime = timer.milliseconds();
                    }

                    if(armGoingUp && Math.abs(timer.milliseconds() - triggerTime) >= 500) {
                        wobbleArm.activate(1.0);
                        armGoingUp = false;
                    } else if(armGoingDown && Math.abs(timer.milliseconds() - triggerTime) >= 500) {
                        wobbleArm.activateClaw(0.0);
                        armGoingDown = false;
                    } else if(armGoingMid && Math.abs(timer.milliseconds() - triggerTime) >= 500) {
                        wobbleArm.activateClaw(0.0);
                        armGoingMid = false;
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    // break out of auto aim
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = DriveMode.DRIVER_CONTROL;
                    }

                    // return control to drivers
                    if (!drive.isBusy()) {
                        currentMode = DriveMode.DRIVER_CONTROL;
                    }
                    break;
            }

            // telemetry logging
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading (imu) ", Math.toDegrees(getIMUHeading()));
            telemetry.addData("hopper", shooter.getHopperHeight());
            telemetry.addLine("A: " + Integer.toString(shooterCounterA) + " B: " + Integer.toString(shooterCounterB));
            telemetry.update();
        }
    }

    public double gamepadScaling(double gamepadInput) {
        //return Math.copySign(Math.pow(gamepadInput, 2), gamepadInput);
        double TANH_SCALE = 2.7;
        return Math.tanh(TANH_SCALE * gamepadInput);
    }

    private double getIMUHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double deltaAngle =  initialAngle + angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -Math.PI) {
            deltaAngle += 2 * Math.PI;
        } else if (deltaAngle > Math.PI) {
            deltaAngle -= 2 * Math.PI;
        }

        return deltaAngle;
    }

    public void shootRings(int nRings) {
        for(int i=0; i < nRings; i++) {
            flicker.activate();
            sleep(120);
        }

        shooterCounterA++;
    }
}
