package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.definitions.Team;

@Autonomous(name = "BlueBasicVikingsAutonomous", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends Base
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }

    public static class Paths {

        public PathChain ToShoot;
        public PathChain Ball1;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        private Follower follower;

        private Timer timer, opModeTimer;

        public enum PathState {
            START_TO_SHOOT,
            LOAD_SHOOT,


        }

        PathState pathState;

            final Pose startPose = new Pose(124.000, 123.000, Math.toRadians(37));
            final Pose shootPose = new Pose(90.0, 97.2, Math.toRadians(37));
            final Pose ball1Pose = new Pose(90.0, 84.0, Math.toRadians(-90));
            final Pose path3Pose = new Pose(125.0, 84.0, Math.toRadians(37));
            final Pose path4Pose = new Pose(90.0, 97.2, Math.toRadians(37));



        public Paths(Follower follower) {
            ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose,shootPose)
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Ball1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 97.200), new Pose(90.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 84.000), new Pose(125.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 84.000), new Pose(90.000, 97.200))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 97.200), new Pose(90.000, 58.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 58.000), new Pose(130.000, 58.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.000, 58.000), new Pose(90.000, 97.200))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 97.200), new Pose(90.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 35.000), new Pose(135.771, 34.971))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135.771, 34.971), new Pose(90.000, 97.200))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(90.000, 97.200), new Pose(84.171, 43.543))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

}