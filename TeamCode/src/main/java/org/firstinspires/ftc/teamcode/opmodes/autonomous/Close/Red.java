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

@Autonomous(name = "CloseRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends AutoBase
{
    private Paths paths;

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.RED;

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
                    follower.followPath(paths.finalPose, false);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose:
                    finishedAutonomous = true;
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
                case ToShoot:
                    // Shoot, then go to pre-ball-one
                    Shoot();
                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                case ToBallOne: // At shooting position, going to ball one
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

                case ToShoot_1: // At ball one full, going back to shooting position
                    Shoot();

                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;

                case ToBallTwo: // At shooting position, going to ball two
                    startIntake();

                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;

                case ToBallTwoFull: // At ball two, going to ball two full
                    stopIntake();

                    follower.followPath(paths.ToBallTwoBack);
                    setPathState(Paths.PathState.ToBallTwoBack);
                    break;

                case ToBallTwoBack:
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;

                case ToShoot_2: // At ball two full, going back to shooting position
                    Shoot();

                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;

                case ToThree: // At shooting position, going to ball three

                    startIntake();
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;

                case ToThreeFull: // At ball three, going to ball three full
                    stopIntake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3: // At ball three full, going back to shooting position
                    Shoot();

                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose: // At shooting position, going to final pose
                    finishedAutonomous = true;
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
                case ToShoot: // Going to shoot
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                case ToBallOne: // At shooting position, going to ball one
                    Shoot();

                    startIntake();
                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;

                case ToBallOneFull: // At ball one, going to ball one full
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;

                case ToShoot_1: // At ball one full, going back to shooting position
                    stopIntake();

                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.Gate);
                    break;

                case Gate: // At shooting position, going to gate
                    Shoot();

                    startIntake();
                    follower.followPath(paths.Gate);
                    setPathState(Paths.PathState.Eat);
                    break;

                case Eat: // At gate, going to eat
                    follower.followPath(paths.Eat);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;

                case ToShoot_2: // At eat, going back to shooting position
                    stopIntake();

                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.Gate_2);
                    break;

                case Gate_2: // At shooting position, going to gate 2
                    Shoot();

                    startIntake();
                    follower.followPath(paths.Gate_2);
                    setPathState(Paths.PathState.Eat_2);
                    break;

                case Eat_2: // At gate 2, going to eat 2
                    follower.followPath(paths.Eat_2);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3: // At eat 2, going back to shooting position
                    stopIntake();

                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.Gate_3);
                    break;

                case Gate_3: // At shooting position, going to gate 3
                    Shoot();

                    startIntake();
                    follower.followPath(paths.Gate_3);
                    setPathState(Paths.PathState.Eat_3);
                    break;

                case Eat_3: // At gate 3, going to eat 3
                    follower.followPath(paths.Eat_3);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;

                case ToShoot_4: // At eat 3, going back to shooting position
                    stopIntake();

                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.bottomBalls);
                    break;

                case bottomBalls: // At shooting position, going to bottom balls
                    Shoot();

                    startIntake();
                    follower.followPath(paths.bottomBalls);
                    setPathState(Paths.PathState.bottomBallsEat);
                    break;

                case bottomBallsEat: // At bottom balls, going to eat
                    follower.followPath(paths.bottomBallsEat);
                    setPathState(Paths.PathState.ToShoot_5);
                    break;

                case ToShoot_5: // At bottom balls eat, going back to shooting position
                    stopIntake();

                    follower.followPath(paths.ToShoot_5);
                    setPathState(Paths.PathState.upperBalls);
                    break;

                case upperBalls: // At shooting position, going to upper balls
                    Shoot();

                    startIntake();
                    follower.followPath(paths.upperBalls);
                    setPathState(Paths.PathState.upperEat);
                    break;

                case upperEat: // At upper balls, going to eat
                    follower.followPath(paths.upperEat);
                    setPathState(Paths.PathState.upperTurn);
                    break;

                case upperTurn: // At upper eat, going to turn
                    stopIntake();

                    follower.followPath(paths.upperTurn);
                    setPathState(Paths.PathState.toShoot);
                    break;

                case toShoot: // At upper turn, going back to shooting position
                    follower.followPath(paths.toShoot);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose: // At shooting position, going to final pose
                    Shoot();

                    follower.followPath(paths.finalPose, false);
                    break;
            }
        }
    }


    static class Paths
    {
        public Pose startPose = new Pose(21.000, 125.000, Math.toRadians(0));

        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallOneBack, ToBallTwo, ToBallTwoFull, ToBallTwoBack, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
        // Gate-specific paths used in buildPathsGate
        Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, finalPose;

        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToBallOneBack, ToBallTwo, ToBallTwoFull, ToBallTwoBack, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
            // Gate-specific states
            Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, bottomBalls, bottomBallsEat, ToShoot_5, upperEat, upperBalls, upperTurn, toShoot, finalPose,
        }


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
            startPose = new Pose(79, 9,  Math.toRadians(90));
            final Pose shootPose = new Pose(79, 9);
            final Pose leavePose = new Pose(85, 50);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,

                                    shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            finalPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    leavePose))
                    .setConstantHeadingInterpolation(Math.toRadians(142))

                    .build();
        }

        private void buildPathsReg(Follower follower)
        {
            // --- Pose definitions ---
            startPose = new Pose(79, 9, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(86, 36);
            final Pose ballOneFullPose = new Pose(125, 36);
            final Pose shootPose = new Pose(95, 100.5);

            final Pose ballTwoLinePose = new Pose(87, 60.000);
            final Pose ballTwoFullPose = new Pose(125, 60.000);

            final Pose ballThreeLinePose = new Pose(87, 84);
            final Pose ballThreeFullPose = new Pose(125, 84);

            final Pose finalPose = new Pose(96, 78);

            final Pose shootThreeLinePose = new Pose(90, 84);

            // --- Paths ---
            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            ToBallOneFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setLinearHeadingInterpolation(ballOneLinePose.getHeading(), Math.toRadians(0))
                    .build();
            ToBallOneBack =  follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, ballOneLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .build();

            ToBallTwo = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .build();

            ToBallTwoFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoLinePose, ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();
            ToBallTwoBack = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoFullPose, ballTwoLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            ToShoot_2 = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoLinePose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .build();
            ToThree = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(36),Math.toRadians(0))
                    .build();

            ToThreeFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeLinePose, ballThreeFullPose))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            ToShoot_3 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeFullPose, shootThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                    .build();

            this.finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootThreeLinePose, finalPose))
                    .setConstantHeadingInterpolation(Math.toRadians(44))
                    .build();
        }


        private void buildPathsGate(Follower follower)
        {

            startPose = new Pose(17.000, 127.000,  Math.toRadians(145));
            final Pose shootPose = new Pose(71.000, 71.000);
            final Pose ballOneLinePose = new Pose(56.000, 60.000);
            final Pose ballOneFullPose = new Pose(56.000, 60.000);
            final Pose gateLinePose = new Pose(15.000, 71.000);
            final Pose gateControlPoint = new Pose(20.000, 64.000);
            final Pose eatLinePose = new Pose(10.000, 61.000);
            final Pose ballThreePose = new Pose(71.000, 35.000);
            final Pose ballThreeFullPose = new Pose(15.000, 35.000);
            final Pose ballPickPose = new Pose(58.000, 84.000);
            final Pose TurnPose = new Pose(50.000, 84.000);
            final Pose topEatPose = new Pose(16.000, 84.000);
            final Pose randomPose = new Pose(28.000, 84.000);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,
                                    shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToBallOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneLinePose,
                                    ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneFullPose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    gateLinePose))
                    .setTangentHeadingInterpolation()
                    .build();

            Eat = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateLinePose,
                                    gateControlPoint,
                                    eatLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    eatLinePose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))
                    .build();

            Gate_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    gateLinePose))
                    .setTangentHeadingInterpolation()
                    .build();

            Eat_2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateLinePose,
                                    gateControlPoint,
                                    eatLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            ToShoot_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    eatLinePose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))
                    .build();

            Gate_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    gateLinePose))
                    .setTangentHeadingInterpolation()
                    .build();

            Eat_3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateLinePose,
                                    gateControlPoint,
                                    eatLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            ToShoot_4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    eatLinePose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))
                    .build();

            bottomBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,
                                    ballThreePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            bottomBallsEat = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreePose,
                                    ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeFullPose,
                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();
            upperBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballPickPose))
                    .setTangentHeadingInterpolation()

                    .build();

            upperEat = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballPickPose,

                                    TurnPose))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            upperTurn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    TurnPose,

                                    topEatPose))
                    .setTangentHeadingInterpolation()

                    .build();

            toShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    topEatPose,

                                    ballPickPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            finalPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballPickPose,

                                    randomPose))
                    .setTangentHeadingInterpolation()

                    .build();
        }
    }
}
