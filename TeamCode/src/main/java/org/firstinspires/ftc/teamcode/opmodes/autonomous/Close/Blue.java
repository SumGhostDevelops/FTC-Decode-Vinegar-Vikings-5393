package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;


@Autonomous(name = "CloseBlueAuto", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends AutoBase
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
    protected Object createPaths(Follower follower, AutoStrat autoStrat)
    {
        return new Paths(follower, autoStrat);
    }

    @Override
    protected Pose getStartingPose(Object paths)
    {
        return ((Paths) paths).startPose;
    }

    private void setPathState(Paths.PathState pathState)
    {
        setPathState((Object) pathState);
    }

    private Paths.PathState getPathState()
    {
        return (Paths.PathState) currentPathState;
    }

    @Override
    protected void PathBasic()
    {
        if (!follower.isBusy())
        {
            switch (getPathState())
            {
                case ToShoot:
                    //Shoots first, then drives to final pose
                    Shoot();

                    //at starting position, starts driving to position 2.
                    follower.followPath(paths.FinalPose, false);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose:
                    autonomousFinished = true;
                    break;
            }
        }
    }

    @Override
    protected void PathRegular()
    {

        if (!follower.isBusy())
        {
            switch (getPathState())
            {
                //  When in a state, start the NEXT path.
                case ToShoot: // Shoot, then go to pre-ball-one
                    Shoot();

                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                case ToBallOne: // Get ready to pick up balls
                    startIntake();

                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;

                case ToBallOneFull: // Picked up all the balls, now drive back
                    stopIntake();

                    follower.followPath(paths.ToBallOneBack);
                    setPathState(Paths.PathState.ToBallOneBack);
                    break;

                case ToBallOneBack: // Drive to shooting position
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;

                case ToShoot_1: // At shoot position; shoot and drive to pre ball two
                    Shoot();

                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;

                case ToBallTwo: // Pick up the balls
                    startIntake();

                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;

                case ToBallTwoFull:  // Picked up the balls, drive to shoot
                    stopIntake();
                    follower.followPath(paths.ToBallTwoBack);
                    setPathState(Paths.PathState.ToBallTwoBack);
                    break;
                case ToBallTwoBack:  // Picked up the balls, drive to shoot

                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;


                case ToShoot_2: // Shoot, then drive to three
                    Shoot();

                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;

                case ToThree: // Pick up the balls
                    startIntake();
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;

                case ToThreeFull: // Picked up balls, drive to shoot
                    stopIntake();

                    follower.followPath(paths.bottomBalls);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3: // Shoot, then drive to final pose
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
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
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
                    follower.followPath(paths.Gate);
                    setPathState(Paths.PathState.Gate);
                    break;

                case Gate:
                    follower.followPath(paths.Eat);
                    setPathState(Paths.PathState.Eat);
                    break;

                case Eat:
                    stopIntake();
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;

                case ToShoot_1:
                    Shoot();
                    follower.followPath(paths.Gate_2);
                    setPathState(Paths.PathState.Gate_2);
                    break;

                case Gate_2:
                    startIntake();
                    follower.followPath(paths.Eat_2);
                    setPathState(Paths.PathState.Eat_2);
                    break;

                case Eat_2:
                    follower.followPath(paths.Gate_3);
                    setPathState(Paths.PathState.Gate_3);
                    break;

                case Gate_3:
                    follower.followPath(paths.Eat_3);
                    setPathState(Paths.PathState.Eat_3);
                    break;

                case Eat_3:
                    stopIntake();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;

                case ToShoot_2:
                    Shoot();
                    follower.followPath(paths.bottomBalls);
                    setPathState(Paths.PathState.bottomBalls);
                    break;

                case bottomBalls:
                    startIntake();
                    follower.followPath(paths.bottomBallsEat);
                    setPathState(Paths.PathState.bottomBallsEat);
                    break;

                case bottomBallsEat:
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3:
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;

                case ToShoot_4:
                    stopIntake();
                    follower.followPath(paths.ToShoot_5);
                    setPathState(Paths.PathState.ToShoot_5);
                    break;

                case ToShoot_5:
                    Shoot();
                    follower.followPath(paths.upperBalls);
                    setPathState(Paths.PathState.upperBalls);
                    break;

                case upperBalls:
                    startIntake();
                    follower.followPath(paths.upperEat);
                    setPathState(Paths.PathState.upperEat);
                    break;

                case upperEat:
                    stopIntake();
                    follower.followPath(paths.toShoot);
                    setPathState(Paths.PathState.toShoot);
                    break;

                case toShoot:
                    Shoot();
                    break;
            }
        }
    }

    static class Paths
    {

        public Pose startPose = new Pose(123, 125, Math.toRadians(90));
        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallTwoBack,ToBallOneBack,ballBallsShoot, ToBallTwo, ToBallTwoFull, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
        // Gate-specific paths used in buildPathsGate
        Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, FinalPose, finalPose;

        public Paths(Follower follower, AutoStrat strat)
        {

            switch (strat)
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
            // Tune these poses if the robot drives too far or in the wrong direction.
            startPose = new Pose(80, 8.3, Math.toRadians(90));
            final Pose shootPose = new Pose(80, 8.3);
            final Pose finalPose = new Pose(80, 50);

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setTangentHeadingInterpolation()
                    .build();

            FinalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, finalPose))
                    .setConstantHeadingInterpolation(Math.toRadians(142))
                    .build();

        }

        private void buildPathsGate(Follower follower)
        {
            startPose = new Pose(62.000, 8.000, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(52.000, 59.000);
            final Pose ballOneFullPose = new Pose(19.000, 60.000);
            final Pose gateLinePose = new Pose(16.000, 70.000);
            final Pose gateControlPoint = new Pose(35.000, 60.000);
            final Pose eatLinePose = new Pose(9.000, 55.000);
            final Pose ballThreePose = new Pose(62.000, 36.000);
            final Pose ballThreeFullPose = new Pose(25.000, 70.000);
            final Pose ballPickPose = new Pose(52.000, 36.000);
            final Pose topEatPose = new Pose(17.000, 36.000);
            final Pose randomPose = new Pose(63.000, 20.000);
            final Pose shootPose = new Pose(62.000, 8.000);

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            Gate = follower.pathBuilder()
                    .addPath(new BezierCurve(ballOneFullPose, gateControlPoint, gateLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Eat = follower.pathBuilder()
                    .addPath(new BezierLine(gateLinePose, ballThreePose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreePose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Gate_2 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballThreeFullPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Eat_2 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeFullPose, gateLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Gate_3 = follower.pathBuilder()
                    .addPath(new BezierCurve(gateLinePose, gateControlPoint, eatLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Eat_3 = follower.pathBuilder()
                    .addPath(new BezierLine(eatLinePose, ballThreePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .setReversed()
                    .build();

            ToShoot_2 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreePose, shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            bottomBalls = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballThreeFullPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            bottomBallsEat = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeFullPose, gateLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            ToShoot_3 = follower.pathBuilder()
                    .addPath(new BezierCurve(gateLinePose, gateControlPoint, eatLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            ToShoot_4 = follower.pathBuilder()
                    .addPath(new BezierLine(eatLinePose, ballThreePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            ToShoot_5 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreePose, shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            upperBalls = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballPickPose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            upperEat = follower.pathBuilder()
                    .addPath(new BezierLine(ballPickPose, topEatPose))
                    .setTangentHeadingInterpolation()
                    .build();

            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(topEatPose, randomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }

        private void buildPathsReg(Follower follower)
        {

            // --- Pose definitions ---
            startPose = new Pose(60.500, 9.000, Math.toRadians(90));
            final Pose ballOneLinePose = new Pose(55.000, 36.000);
            final Pose ballOneFullPose = new Pose(19.000, 36.000);
            final Pose shootPose = new Pose(48, 95.200);

            final Pose ballTwoLinePose = new Pose(55.000, 60.000);
            final Pose ballTwoFullPose = new Pose(19.000, 60.000);

            final Pose ballThreeLinePose = new Pose(55.000, 84.000);
            final Pose ballThreeFullPose = new Pose(19.000, 84.000);

            final Pose gateLinePose = new Pose(54.000, 84.000);
            final Pose gatePushPose = new Pose(49.000, 79.000);
            final Pose randomPose = new Pose(55, 65);


            ToBallOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,
                                    ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneLinePose,
                                    ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();
            ToBallOneBack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneFullPose,
                                    ballOneLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneLinePose,
                                    shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();

            ToBallTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoLinePose,
                                    ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();
            ToBallTwoBack = follower.pathBuilder()
                    .addPath(new BezierLine(
                            ballTwoFullPose, ballTwoLinePose)
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoLinePose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            ToThree = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            ToThreeFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeLinePose,
                                    ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            bottomBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeFullPose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                    .build();
            ballBallsShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    gateLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                    .build();
            bottomBallsEat = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateLinePose,
                                    gatePushPose))
                    .setConstantHeadingInterpolation(Math.toRadians(136))
                    .build();
            finalPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateLinePose,
                                    randomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(136))
                    .build();
        }

        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToShootFar, ToBallTwoBack, ToBallTwo, ToBallTwoFull,bottomBalls,  bottomBallsShoot,Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
            // Gate-specific states
            Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, bottomBallsEat, ToShoot_5, upperEat, upperBalls, upperTurn, toShoot, finalPose, ToBallOneBack, FinalPose
        }
    }
}