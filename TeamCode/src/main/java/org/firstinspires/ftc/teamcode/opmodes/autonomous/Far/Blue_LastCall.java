package org.firstinspires.ftc.teamcode.opmodes.autonomous.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;

@Autonomous(name = "FarBlue_LastCall", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue_LastCall extends AutoBase
{
    private Paths paths;

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;

        // Strategy selection
        autoStrat = selectStrategy();

        // Initialize autonomous
        follower = PedroConstants.createFollower(hardwareMap);
        setFollower(follower);
        paths = new Paths(follower, autoStrat);
        initAuto(paths);

        // Set initial state
        setPathState(Paths.PathState.ToShoot);

        // Wait for start
        waitForStart();

        // Run the main autonomous loop
        runAutoLoop();
    }
    @Override
    protected StrategyOption[] getStrategyOptions()
    {
        return new StrategyOption[]{
                new StrategyOption(AutoStrat.REGULAR, "DPAD RIGHT", "12 Ball Gate (REGULAR)"),
        };
    }

    @Override
    protected Object createPaths(Follower follower, AutoStrat autoStrat)
    {
        return new Paths(follower, autoStrat);
    }

    @Override
    protected Pose getStartingPose(Object paths)
    {
        return ((Paths) paths).startPose;
    }

    @Override
    protected void PathBasic()
    {

    }

    private Paths.PathState getPathState()
    {
        return (Paths.PathState) currentPathState;
    }

    private void setPathState(Paths.PathState pathState)
    {
        setPathState((Object) pathState);
    }

    @Override
    protected void PathRegular()
    {
        if (!follower.isBusy())
        {
            switch (getPathState())
            {
                case ToShoot:
                    startOuttake();
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.ToShootFar);
                    break;

                case ToShootFar:
                    Shoot();
                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                case ToBallOne:
                    startIntake();
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;

                case ToBallOneFull:
                    stopIntake();
                    follower.followPath(paths.Gate);
                    setPathState(Paths.PathState.Gate);
                    break;

                case Gate:
                    follower.followPath(paths.Gate_2);
                    setPathState(Paths.PathState.Gate_2);
                    break;

                case Gate_2:
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;

                case ToShoot_1:
                    Shoot();
                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;

                case ToBallTwo:
                    startIntake();
                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;

                case ToBallTwoFull:
                    stopIntake();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;

                case ToShoot_2:
                    Shoot();
                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;

                case ToThree:
                    startIntake();
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;

                case ToThreeFull:
                    stopIntake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3:
                    Shoot();
                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose:
                    autonomousFinished = true;
                    break;
            }
        }
    }

    @Override
    protected void PathGate()
    {

    }

    public static class Paths
    {

        public Pose startPose = new Pose(21.000, 125.000, Math.toRadians(145));

        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
        // Gate-specific paths used in buildPathsGate
        Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, finalPose;

        public Paths(Follower follower, AutoStrat autoStrat)
        {
            switch (autoStrat)
            {

                case REGULAR:
                default:
                    buildPathsReg(follower);
                    break;
            }

        }



        private void buildPathsReg(Follower follower)
        {
            startPose = new Pose(20.000, 123.000, Math.toRadians(145));

            final Pose ballOneLinePose = new Pose(54.000, 84.000);
            final Pose ballOneFullPose = new Pose(18.000, 84.000);
            final Pose gateHitPose = new Pose(16.000, 69.000);
            final Pose gateControlPoint = new Pose(35.000, 65.000);

            final Pose shootPose = new Pose(54.000, 97.200);

            final Pose ballTwoLinePose = new Pose(54.000, 60.000);
            final Pose ballTwoFullPose = new Pose(17.000, 60.000);

            final Pose ballThreeLinePose = new Pose(54.000, 35.000);
            final Pose ballThreeFullPose = new Pose(16.000, 35.000);

            final Pose gateLinePose = new Pose(54.000, 69.000);
            final Pose finalPose = new Pose(53.890, 69.156);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,
                                    shootPose
                            )).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

// Ball Two first
            ToBallOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoLinePose,
                                    ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    ballTwoFullPose,
                                    gateControlPoint,
                                    gateHitPose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Gate_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateHitPose,
                                    gateLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateLinePose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

// Ball Three second
            ToBallTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneLinePose,
                                    ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneFullPose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

// Ball One last
            ToThree = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToThreeFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeLinePose,
                                    ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeFullPose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            this.finalPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    finalPose))
                    .setConstantHeadingInterpolation(Math.toRadians(143))
                    .build();
        }

        public enum PathState
        {
            ToShoot, ToShootFar, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
            // Gate-specific states
            Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, bottomBalls, bottomBallsEat, ToShoot_5, upperEat, upperBalls, upperTurn, toShoot, finalPose,
        }
    }
}