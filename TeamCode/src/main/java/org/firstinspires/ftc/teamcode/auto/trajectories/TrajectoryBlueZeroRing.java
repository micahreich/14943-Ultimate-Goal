package org.firstinspires.ftc.teamcode.auto.trajectories;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;

public class TrajectoryBlueZeroRing {
    public enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        TRAJECTORY_4,
        TRAJECTORY_5
    }

    public ArrayList<Trajectory> oneRingTrajectoryList = new ArrayList<Trajectory>();

    public TrajectoryBlueZeroRing() {
       // pass
    }
}
