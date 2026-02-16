package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close.CompClose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;

@Autonomous(name = "ShootingCloseBlueAuto", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class BlueComp extends AutoBase
{
    private Paths paths;

    @Override
    protected StrategyOption[] getStrategyOptions()
    {
        return new StrategyOption[]{
                new StrategyOption(AutoStrat.BASIC, "DPAD UP", "3 Ball (BASIC)"),
                new StrategyOption(AutoStrat.REGULAR, "DPAD RIGHT", "DOESNT WORK !! 9 Ball (REGULAR)")
        };
    }

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
        if (!follower.isBusy())
        {
            switch (getPathState())
            {

                case ToShoot: // Going to shoot
                    Shoot();
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                case ToBallOne: // At shooting position, going to ball one


                    startIntake();
                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;


                case ToShoot_1: // At ball one full, going back to shooting position
                    Shoot();

                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose: // At shooting position, going to final pose
                    follower.followPath(paths.finalPose);

                    break;
            }
        }
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
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
        if (!follower.isBusy())
        {
            switch (getPathState())
            {

                case ToShoot: // Going to shoot
                    Shoot();
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                case ToBallOne: // At shooting position, going to ball one
                    startIntake();
                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;


                case ToShoot_1: // At ball one full, going back to shooting position
                    stopIntake();
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose: // At shooting position, going to final pose
                    Shoot();
                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.FinalPos);
                    break;
                case FinalPos:
                    finishedAutonomous = true;
                    break;
            }
        }
    }

    @Override
    protected void PathGate()
    {
        // Not implemented for this autonomous
    }


    static class Paths
    {
        public Pose startPose = new Pose(21.000, 125.000, Math.toRadians(0));

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
                case BASIC:
                    buildPathsBasic(follower);
                    break;
                case GATE:
                    buildPathsGate(follower);
                    break;
                case REGULAR:
                default:
                    buildPathsReg(follower);
                    break;
            }

        }

        private void buildPathsBasic(Follower follower)
        {
            // --- Pose definitions ---
            startPose = new Pose(65, 9, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(8, 26);
            final Pose ballOneFullPose = new Pose(8, 9);

            final Pose shootPose = new Pose(65, 9);

            final Pose randomPose = new Pose(24, 12);

            // --- Paths ---
            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                    .build();

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                    .build();


            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, randomPose))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        private void buildPathsGate(Follower follower)
        {

        }

        private void buildPathsReg(Follower follower)
        {
            // --- Pose definitions ---
            startPose = new Pose(65, 9, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(48, 36);
            final Pose ballOneFullPose = new Pose(14, 36);

            final Pose shootPose = new Pose(65, 9);

            final Pose randomPose = new Pose(31, 11);

            // --- Paths ---
            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, randomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(44))
                    .build();
        }


        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
            // Gate-specific states
            Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, bottomBalls, bottomBallsEat, ToShoot_5, upperEat, upperBalls, upperTurn, toShoot, finalPose, FinalPos
        }
    }
}