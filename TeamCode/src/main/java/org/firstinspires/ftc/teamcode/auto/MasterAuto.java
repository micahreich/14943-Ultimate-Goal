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

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemCamera;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemFlicker;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemShooter;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemWobbleArm;

import java.util.Arrays;

@TeleOp(group = "drive", name = "MASTER AUTO")
public class MasterAuto extends LinearOpMode {
    SubsystemShooter shooter;
    SubsystemFlicker flicker;
    SubsystemWobbleArm wobbleArm;
    SubsystemIntake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        SubsystemCamera vision = new SubsystemCamera(this, hardwareMap, telemetry);
        vision.initialize();

        SubsystemCamera.stackHeight finalHeight = SubsystemCamera.stackHeight.ZERO;

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

        /* ------------
        ZERO RING TRAJECTORIES
        ------------ */
        // TRAJ: shoot preloaded rings
        Trajectory R0_toShoot_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-5, 41, Math.toRadians(180)))
                .addTemporalMarker(0.05, 0.0, () -> {
                    shooter.activate(1740);
                })
                .build();

        // TRAJ: release wobble 1
        Trajectory R0_toZoneC_1 = drive.trajectoryBuilder(R0_toShoot_1.end())
                .lineToLinearHeading(new Pose2d(-5, 53, Math.toRadians(0)))
                .build();

        // TRAJ: grab wobble 2
        Trajectory R0_toWobble_p1 = drive.trajectoryBuilder(R0_toZoneC_1.end())
                .lineToLinearHeading(new Pose2d(-21.5, 57, Math.toRadians(180)))
                .addTemporalMarker(0.0, 0.0, () -> {
                    wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.UPRIGHT);
                })
                .addTemporalMarker(0.9, 0.0, () -> {
                    wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER);
                })
                .build();

        Trajectory R0_toWobble_p2 = drive.trajectoryBuilder(R0_toWobble_p1.end())
                .lineToLinearHeading(new Pose2d(-34, 54, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // TRAJ: release wobble 2
        Trajectory R0_toZoneC_2_p1 = drive.trajectoryBuilder(R0_toWobble_p2.end())
                .lineToLinearHeading(new Pose2d(-20, 59, Math.toRadians(180)))
                .build();

        Trajectory R0_toZoneC_2_p2 = drive.trajectoryBuilder(R0_toZoneC_2_p1.end())
                .lineToLinearHeading(new Pose2d(-12, 59, Math.toRadians(0)))
                .build();

        // to park line
        Trajectory R0_toPark_p1 = drive.trajectoryBuilder(R0_toZoneC_2_p2.end())
                .back(12)
                .build();

        Trajectory R0_toPark_p2 = drive.trajectoryBuilder(R0_toPark_p1.end())
                .lineToLinearHeading(new Pose2d(3, 25, Math.toRadians(90)))
                .build();

        /* ------------
        ONE RING TRAJECTORIES
        ------------ */
        // TRAJ: shoot preloaded rings
        Trajectory R1_toShoot_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-3, 18.5, Math.toRadians(200)))
                .addTemporalMarker(0.1, 0.0, () -> {
                    shooter.activate(1690);
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
                .lineToLinearHeading(new Pose2d(-24, 35, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();

        // to shoot last rings
        Trajectory R1_toShoot_last = drive.trajectoryBuilder(R1_toStack_p2.end())
                .lineToLinearHeading(new Pose2d(-5, 35, Math.toRadians(186)))
                .build();

        // to park line
        Trajectory R1_toPark = drive.trajectoryBuilder(R1_toShoot_last.end())
                .lineToLinearHeading(new Pose2d(3, 35, Math.toRadians(0)))
                .build();

        /* ------------
        FOUR RING TRAJECTORIES
        ------------ */
        // TRAJ: shoot preloaded rings
        Trajectory R4_toShoot_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-3, 18.5, Math.toRadians(204)))
                .addTemporalMarker(0.1, 0.0, () -> {
                    shooter.activate(1690);
                })
                .build();

        // TRAJ: release wobble 1
        Trajectory R4_toZoneC_1 = drive.trajectoryBuilder(R4_toShoot_1.end())
                .lineToLinearHeading(new Pose2d(43, 53, Math.toRadians(0)))
                .build();

        // TRAJ: grab wobble 2
        Trajectory R4_toWobble_p1 = drive.trajectoryBuilder(R4_toZoneC_1.end())
                .lineToLinearHeading(new Pose2d(-21.5, 57, Math.toRadians(180)))
                .addTemporalMarker(0.0, 0.0, () -> {
                    wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.UPRIGHT);
                })
                .addTemporalMarker(0.9, 0.0, () -> {
                    wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER);
                })
                .build();

        Trajectory R4_toWobble_p2 = drive.trajectoryBuilder(R4_toWobble_p1.end())
                .lineToLinearHeading(new Pose2d(-34, 56, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15.0, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // TRAJ: release wobble 2
        Trajectory R4_toZoneC_2_p1 = drive.trajectoryBuilder(R4_toWobble_p2.end())
                .lineToLinearHeading(new Pose2d(6, 59, Math.toRadians(180)))
                .build();

        Trajectory R4_toZoneC_2_p2 = drive.trajectoryBuilder(R4_toZoneC_2_p1.end())
                .lineToLinearHeading(new Pose2d(36, 59, Math.toRadians(0)))
                .build();

        // TRAJ: pick up 3/4 stack
        Trajectory R4_toStack_p1 = drive.trajectoryBuilder(R4_toZoneC_2_p2.end())
                .lineToLinearHeading(new Pose2d(-8, 36, Math.toRadians(178)))
                .addTemporalMarker(0.5, 0.0, () -> {
                    shooter.activate(1550);
                })
                .build();

        Trajectory R4_toStack_p2 = drive.trajectoryBuilder(R4_toStack_p1.end())
                .lineToLinearHeading(new Pose2d(-14, 36, Math.toRadians(178)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();

        Trajectory R4_toStack_p3 = drive.trajectoryBuilder(R4_toStack_p2.end())
                .lineToLinearHeading(new Pose2d(-17, 36, Math.toRadians(178)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();

        Trajectory R4_toStack_p4 = drive.trajectoryBuilder(R4_toStack_p3.end())
                .lineToLinearHeading(new Pose2d(-21, 36, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();

        Trajectory R4_toStack_p5 = drive.trajectoryBuilder(R4_toStack_p4.end())
                .lineToLinearHeading(new Pose2d(-32, 38, Math.toRadians(167)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(30))
                .build();

        // to shoot last rings
        Trajectory R4_toShoot_last = drive.trajectoryBuilder(R4_toStack_p5.end())
                .lineToLinearHeading(new Pose2d(-5, 35, Math.toRadians(185)))
                .build();

        // to park line
        Trajectory R4_toPark = drive.trajectoryBuilder(R4_toShoot_last.end())
                .back(10)
                .build();

        while(!isStarted()) {
            finalHeight = vision.getHeight();
            telemetry.addData("Height", vision.getHeight());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        wobbleArm.activateClaw(SubsystemWobbleArm.CLAW_STATE.CLOSED);
        while(opModeIsActive()) {
        switch(finalHeight) {
            case ZERO:
                // shoot preloaded rings
                drive.followTrajectory(R0_toShoot_1);
                shootRings(3);
                shooter.reset();

                // drop wobble 1
                drive.followTrajectory(R0_toZoneC_1);
                sleep(200);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 950,false);
                sleep(250);

                // pick up wobble 2
                drive.followTrajectory(R0_toWobble_p1);
                drive.followTrajectory(R0_toWobble_p2);

                sleep(200);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.CLOSED, 750,true);
                sleep(250);

                // drop wobble 2
                drive.followTrajectory(R0_toZoneC_2_p1);
                drive.followTrajectory(R0_toZoneC_2_p2);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 250,false);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.UPRIGHT,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 0,false);
                sleep(300);

                drive.followTrajectory(R0_toPark_p1);
                drive.followTrajectory(R0_toPark_p2);
                sleep(500);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.RETRACTED,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 0,false);
                sleep(750);
                break;
            case ONE:
                // shoot preloaded rings
                drive.followTrajectory(R1_toShoot_1);
                shootRings(3);
                shooter.reset();

                // drop wobble 1
                drive.followTrajectory(R1_toZoneC_1);
                sleep(200);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 850,false);
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
                break;
            case FOUR:
                // shoot preloaded rings
                drive.followTrajectory(R4_toShoot_1);
                shootRings(3);
                shooter.reset();

                // drop wobble 1
                drive.followTrajectory(R4_toZoneC_1);
                sleep(200);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 750,false);
                sleep(250);

                // pick up wobble 2
                drive.followTrajectory(R4_toWobble_p1);
                drive.followTrajectory(R4_toWobble_p2);

                sleep(200);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.CLOSED, 750,true);
                sleep(250);

                // drop wobble 2
                drive.followTrajectory(R4_toZoneC_2_p1);
                drive.followTrajectory(R4_toZoneC_2_p2);
                wobbleArm.activate(SubsystemWobbleArm.ARM_STATE.LOWER,
                        SubsystemWobbleArm.CLAW_STATE.OPEN, 250,false);

                // pick up 3/4 rings
                drive.followTrajectory(R4_toStack_p1);
                intake.activate(1.0);
                drive.followTrajectory(R4_toStack_p2);
                sleep(500);

                drive.followTrajectory(R4_toStack_p3);
                sleep(500);
                flicker.activate();

                drive.followTrajectory(R4_toStack_p4);
                sleep(500);

                drive.followTrajectory(R4_toStack_p5);
                shooter.activate(1770);
                sleep(500);

                drive.followTrajectory(R4_toShoot_last);
                shootRings2(3);
                // shoot 3/4 rings
                //shootRings(3);
                shooter.reset();
                intake.reset();

                // go to park line
                drive.followTrajectory(R4_toPark);
                break;
        }break;}

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void shootRings(int nRings) {
        for(int i=0; i < nRings + 1; i++) {
            flicker.activate();
            if(i != nRings) {
                sleep(250);
            }
        }
    }

    public void shootRings2(int nRings) {
        for(int i=0; i < nRings + 1; i++) {
            flicker.activate();
            if(i != nRings) {
                sleep(350);
            }
        }
    }
}