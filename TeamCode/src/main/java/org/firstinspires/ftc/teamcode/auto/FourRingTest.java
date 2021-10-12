package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(group = "drive", name = "Four Ring Test")
public class FourRingTest extends LinearOpMode {
    SubsystemShooter shooter;
    SubsystemFlicker flicker;
    SubsystemWobbleArm wobbleArm;
    SubsystemIntake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-63, 39.75, Math.toRadians(180));

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

        Trajectory R4_toShoot_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-7, 59, Math.toRadians(167)))
                .addTemporalMarker(0.1, 0.0, () -> {
                    shooter.activate(SubsystemShooter.SHOOTER_STATE.HIGH_GOAL);
                })
                .build();

        Trajectory R4_toZoneC_1 = drive.trajectoryBuilder(R4_toShoot_1.end())
                .lineToLinearHeading(new Pose2d(51, 48, Math.toRadians(0)))
                .build();

        Trajectory R4_toRing_p1 = drive.trajectoryBuilder(R4_toZoneC_1.end())
                .lineToLinearHeading(new Pose2d(-10, 36.5, Math.toRadians(180)))
                .build();

        Trajectory R4_toRing_p2 = drive.trajectoryBuilder(R4_toRing_p1.end())
                .lineToLinearHeading(new Pose2d(-14, 36.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory R4_toRing_p3 = drive.trajectoryBuilder(R4_toRing_p2.end())
                .lineToLinearHeading(new Pose2d(-18, 36.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory R4_toRing_p4 = drive.trajectoryBuilder(R4_toRing_p3.end())
                .lineToLinearHeading(new Pose2d(-24, 36.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory R4_toShoot_2 = drive.trajectoryBuilder(R4_toRing_p4.end())
                .lineToLinearHeading(new Pose2d(-4, 41, Math.toRadians(180)))
                .build();
        //
        Trajectory R4_toRing_p5 = drive.trajectoryBuilder(R4_toShoot_2.end())
                .lineToLinearHeading(new Pose2d(-21.5, 36, Math.toRadians(180)))
                .build();

        Trajectory R4_toRing_p6 = drive.trajectoryBuilder(R4_toRing_p4.end())
                .lineToLinearHeading(new Pose2d(-30, 35.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory R4_toShoot_3 = drive.trajectoryBuilder(R4_toRing_p6.end())
                .lineToLinearHeading(new Pose2d(-4, 41, Math.toRadians(180)))
                .build();
        //
        Trajectory R4_toWobble_p1 = drive.trajectoryBuilder(R4_toRing_p6.end())
                .lineToLinearHeading(new Pose2d(-34.8, 32.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory R4_toZoneC_2 = drive.trajectoryBuilder(R4_toWobble_p1.end())
                .lineToLinearHeading(new Pose2d(45, 41, Math.toRadians(35)))
                .build();

        Trajectory R4_toParkLine = drive.trajectoryBuilder(R4_toZoneC_2.end())
                .lineToLinearHeading(new Pose2d(17, 47, Math.toRadians(0)))
                .build();

        drive.followTrajectory(R4_toShoot_1);
        while(!shooter.isReady(shooter.shooterPowers.get(SubsystemShooter.SHOOTER_STATE.HIGH_GOAL))) {
            // do nothing
        }
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(200);
        shooter.reset();

        // wobble 1
        drive.followTrajectory(R4_toZoneC_1);
        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 1000,false);

        // pick up ring
        intake.activate(1.0);
        drive.followTrajectory(R4_toRing_p1);
        drive.followTrajectory(R4_toRing_p2);
        sleep(500);
        drive.followTrajectory(R4_toRing_p3);
        sleep(500);
        shooter.activate(1420);
        drive.followTrajectory(R4_toRing_p4);
        sleep(500);

        // shoot ring 2
        // drive.followTrajectory(R4_toShoot_2);
        while(shooter.getHopperHeight() == SubsystemShooter.HOPPER_HEIGHT.ZERO) {
            // do nothing
        }
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(200);

        shooter.activate(1500);
        // ring 4
        //drive.followTrajectory(R4_toRing_p5);
        //sleep(200);
        drive.followTrajectory(R4_toRing_p6);
        sleep(500);
        //sleep(1000);

        // shoot ring 4
        //drive.followTrajectory(R4_toShoot_3);
        while(shooter.getHopperHeight() == SubsystemShooter.HOPPER_HEIGHT.ZERO) {
            // do nothing
        }
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(600);
        flicker.activate();
        sleep(600);
        shooter.reset();
        intake.reset();

        drive.followTrajectory(R4_toWobble_p1);
        sleep(400);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.CLOSED, 850,true);
        sleep(200);

        drive.followTrajectory(R4_toZoneC_2);
        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.RETRACTED,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 650,true);

        drive.followTrajectory(R4_toParkLine);
        //wobble2
        /*sleep(200);
        intake.activate(1.0);

        drive.followTrajectory(R4_toRing_p5);
        sleep(200);
        drive.followTrajectory(R4_toRing_p6);
        sleep(800);

        drive.followTrajectory(R4_toWobble_p1);
        sleep(400);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.CLOSED, 850,true);
        sleep(200);

        drive.followTrajectory(R4_toZoneC_2);
        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.RETRACTED,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 650,true);

        drive.followTrajectory(R4_toParkLine);
        /*wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                           SubsystemWobbleArm.CLAW_STATE.OPEN, 850,false);

        sleep(200);
        drive.followTrajectory(R0_toWobble_p1);
        drive.followTrajectory(R0_toWobble_p2);
        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                SubsystemWobbleArm.CLAW_STATE.CLOSED, 850,true);

        sleep(200);
        drive.followTrajectory(R0_toZoneA_2);
        sleep(200);
        wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.RETRACTED,
                SubsystemWobbleArm.CLAW_STATE.OPEN, 450,true);
        sleep(1000);*/
    }
}
