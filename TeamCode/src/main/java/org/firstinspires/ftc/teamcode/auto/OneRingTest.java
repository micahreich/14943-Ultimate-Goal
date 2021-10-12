package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemFlicker;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemShooter;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemWobbleArm;

import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name = "One Ring Test")
public class OneRingTest extends LinearOpMode {
    SubsystemShooter shooter;
    SubsystemFlicker flicker;
    SubsystemWobbleArm wobbleArm;
    SubsystemIntake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-63, 21.2, Math.toRadians(180));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();

        shooter = new SubsystemShooter(hardwareMap.get(DcMotorEx.class, "shooter1"),
                hardwareMap.get(DcMotorEx.class, "shooter2"),
                hardwareMap.get(DistanceSensor.class, "dist"));

        flicker = new SubsystemFlicker(hardwareMap.get(Servo.class, "flicker"), timer);

        wobbleArm = new SubsystemWobbleArm(hardwareMap.get(Servo.class, "wobble1"),
                hardwareMap.get(Servo.class, "wobble2"),
                hardwareMap.get(Servo.class, "wobbleClaw"), timer);

        intake = new SubsystemIntake(hardwareMap.get(DcMotor.class, "intake1"),
                hardwareMap.get(DcMotor.class, "intake2"));

        waitForStart();

        if (isStopRequested()) return;

        wobbleArm.activateClaw(SubsystemWobbleArm.CLAW_STATE.CLOSED);

        // TRAJ: shoot preloaded rings
        Trajectory R1_toShoot_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-3, 18.5, Math.toRadians(200)))
                .addTemporalMarker(0.1, 0.0, () -> {
                    shooter.activate(1720);
                })
                .build();

        // TRAJ: release wobble 1
        Trajectory R1_toZoneC_1 = drive.trajectoryBuilder(R1_toShoot_1.end())
                .lineToLinearHeading(new Pose2d(19, 29, Math.toRadians(0)))
                .build();

        // TRAJ: grab wobble 2
        Trajectory R1_toWobble_p1 = drive.trajectoryBuilder(R1_toZoneC_1.end())
                .lineToLinearHeading(new Pose2d(-21.5, 57, Math.toRadians(180)))
                .addTemporalMarker(0.0, 0.0, () -> {
                    wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.UPRIGHT);
                })
                .addTemporalMarker(0.9, 0.0, () -> {
                    wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER);
                })
                .build();

        Trajectory R1_toWobble_p2 = drive.trajectoryBuilder(R1_toWobble_p1.end())
                .lineToLinearHeading(new Pose2d(-34, 55, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // TRAJ: release wobble 2
        Trajectory R1_toZoneC_2_p1 = drive.trajectoryBuilder(R1_toWobble_p2.end())
                .lineToLinearHeading(new Pose2d(6, 59, Math.toRadians(180)))
                .build();

        Trajectory R1_toZoneC_2_p2 = drive.trajectoryBuilder(R1_toZoneC_2_p1.end())
                .lineToLinearHeading(new Pose2d(12, 35, Math.toRadians(0)))
                .build();

        // TRAJ: pick up 3/4 stack
        Trajectory R1_toStack_p1 = drive.trajectoryBuilder(R1_toZoneC_2_p2.end())
                .lineToLinearHeading(new Pose2d(-8, 35, Math.toRadians(180)))
                .addTemporalMarker(0.5, 0.0, () -> {
                    shooter.activate(SubsystemShooter.SHOOTER_STATE.HIGH_GOAL);
                })
                .build();

        Trajectory R1_toStack_p2 = drive.trajectoryBuilder(R1_toStack_p1.end())
                .lineToLinearHeading(new Pose2d(-19, 35, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(20))
                .build();

        // to shoot last rings
        Trajectory R1_toShoot_last = drive.trajectoryBuilder(R1_toStack_p2.end())
                .lineToLinearHeading(new Pose2d(-5, 35, Math.toRadians(190)))
                .build();

        // to park line
        Trajectory R1_toPark = drive.trajectoryBuilder(R1_toShoot_last.end())
                .lineToLinearHeading(new Pose2d(3, 35, Math.toRadians(0)))
                .build();

        // shoot preloaded rings
        drive.followTrajectory(R1_toShoot_1);
        shootRings(3);
        shooter.reset();

        // drop wobble 1
        drive.followTrajectory(R1_toZoneC_1);
        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 750,false);
        sleep(250);

        // pick up wobble 2
        drive.followTrajectory(R1_toWobble_p1);
        drive.followTrajectory(R1_toWobble_p2);

        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.CLOSED, 750,true);
        sleep(250);

        // drop wobble 2
        drive.followTrajectory(R1_toZoneC_2_p1);
        drive.followTrajectory(R1_toZoneC_2_p2);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 250,false);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.UPRIGHT,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 0,false);

        // pick up 3/4 rings
        intake.activate(1.0);
        drive.followTrajectory(R1_toStack_p1);
        drive.followTrajectory(R1_toStack_p2);
        sleep(500);
        drive.followTrajectory(R1_toShoot_last);
        shootRings(3);

        // shoot 3/4 rings
        //shootRings(3);
        shooter.reset();

        // go to park line
        drive.followTrajectory(R1_toPark);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.RETRACTED,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 0,false);
        sleep(750);

    }

    public void shootRings(int nRings) {
        for(int i=0; i < nRings + 1; i++) {
            flicker.activate();
            if(i != nRings) {
                sleep(250);
            }
        }
    }
}
