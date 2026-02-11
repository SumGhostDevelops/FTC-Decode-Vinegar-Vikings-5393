package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;

@Autonomous(name = "CloseRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends AutoBase
{

    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;

    private Paths.PathState currentPathState;

    private AutoStrat autoStrat = AutoStrat.REGULAR;

    @Override
    public void runOpMode() throws InterruptedException
    {

        team = Team.RED;

        while (opModeInInit())
        {
            // --- Move your selection logic inside this loop ---
            if (gamepad1.dpad_up)
            {
                autoStrat = AutoStrat.GATE;
            } else if (gamepad1.dpad_right)
            {
                autoStrat = AutoStrat.REGULAR;
            } else if (gamepad1.dpad_down)
            {
                autoStrat = AutoStrat.BASIC;
            }

            // --- Keep your telemetry inside the loop for live feedback ---
            telemetry.addLine("--- SELECT AUTO STRATEGY ---");
            telemetry.addData("Selected", autoStrat);
            telemetry.addLine("\nControls:");
            telemetry.addLine("DPAD UP: 12 Ball (GATE)");
            telemetry.addLine("DPAD RIGHT: 9 Ball (REGULAR)");
            telemetry.addLine("DPAD DOWN: 3 Ball (BASIC)");
            telemetry.update();
        }

        // Now the selection is locked in. Initialize paths and timers.
        initAuto();

        // The OpMode will wait here until you press START
        waitForStart();

        if (opModeIsActive() && !isStopRequested())
        {
            opModeTimer.resetTimer();
            // follower.followPath(paths.ToShoot);

            while (opModeIsActive() && !isStopRequested())
            {
                handlePathing();
                follower.update();

                updateSubsystems();

                telemetry.addData("Current State", currentPathState);
                telemetry.addData("State Time (s)", timer.getElapsedTimeSeconds());
                telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
                telemetry.update();
            }
        }

    }

    public void initAuto()
    {
        initRobot();

        follower = PedroConstants.createFollower(hardwareMap);
        setFollower(follower);
        paths = new Paths(follower,  autoStrat);
        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower.setStartingPose(paths.startPose);
        follower.update();

        setPathState(Paths.PathState.ToShoot);

    }

    public void setPathState(Paths.PathState pathState)
    {
        currentPathState = pathState;
        timer.resetTimer();
    }

    private void handlePathing()
    {
        switch (autoStrat)
        {

            case BASIC:
               PathBasic();
                break;
            case REGULAR:
                PathRegular();
                break;
            case GATE:
                PathGate();
                break;
        }
    }

    private void PathBasic()
    {
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
        if (!follower.isBusy())
        {

            switch (currentPathState)
            {
                case ToShoot:
                    Shoot();
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.finalPose);
                    break;
                case finalPose:
                    // end path
                    follower.followPath(paths.finalPose);
            }
        }
    }

    private void PathRegular()
    {
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
        if (!follower.isBusy())
        {
            switch (currentPathState)
            {
                // The logic is now: when a state is finished, start the NEXT path and set the
                // NEXT state.
                case ToShoot:
                    Shoot();
                    follower.followPath(paths.ToBallOne); // Start NEXT path
                    setPathState(Paths.PathState.ToBallOne); // Set NEXT state
                    break;
                case ToBallOne:
                    Intake();
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;
                case ToBallOneFull:
                    // has ball
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;
                case ToShoot_1:
                    Shoot();
                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;
                case ToBallTwo:
                    Intake();
                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;
                case ToBallTwoFull:
                    // has ball
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;
                case ToShoot_2:
                    Shoot();
                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;
                case ToThree:
                    Intake();
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;
                case ToThreeFull:
                    // has ball
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;
                case ToShoot_3:
                    Shoot();
                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.finalPose);
                    break;
                case finalPose:
                    // end path
                    break;
            }
        }
    }

    private void PathGate()
    {
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
        if (!follower.isBusy())
        {
            switch (currentPathState)
            {

                // --- Start: go shoot first ---
                case ToShoot:
                    Shoot();
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.ToBallOne);
                    break;

                // --- Ball 1 sequence ---
                case ToBallOne:
                    Intake();
                    follower.followPath(paths.ToBallOne);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;

                case ToBallOneFull:
                    // full ball
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;

                case ToShoot_1:
                    Shoot();
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.Gate);
                    break;

                // --- Gate cycle 1 ---
                case Gate:

                    follower.followPath(paths.Gate);
                    setPathState(Paths.PathState.Eat);
                    break;

                case Eat:
                    // intake
                    follower.followPath(paths.Eat);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;

                case ToShoot_2:
                    Shoot();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.Gate_2);
                    break;

                // --- Gate cycle 2 ---
                case Gate_2:
                    follower.followPath(paths.Gate_2);
                    setPathState(Paths.PathState.Eat_2);
                    break;

                case Eat_2:
                    Intake();
                    follower.followPath(paths.Eat_2);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3:
                    Shoot();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.Gate_3);
                    break;

                // --- Gate cycle 3 ---
                case Gate_3:
                    follower.followPath(paths.Gate_3);
                    setPathState(Paths.PathState.Eat_3);
                    break;

                case Eat_3:
                    Intake();
                    follower.followPath(paths.Eat_3);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;

                case ToShoot_4:
                    Shoot();
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.bottomBalls);
                    break;

                // --- Bottom balls sequence ---
                case bottomBalls:
                    follower.followPath(paths.bottomBalls);
                    setPathState(Paths.PathState.bottomBallsEat);
                    break;

                case bottomBallsEat:
                    Intake();
                    follower.followPath(paths.bottomBallsEat);
                    setPathState(Paths.PathState.ToShoot_5);
                    break;

                case ToShoot_5:
                    Shoot();
                    follower.followPath(paths.ToShoot_5);
                    setPathState(Paths.PathState.upperBalls);
                    break;

                // --- Upper balls sequence ---
                case upperBalls:
                    follower.followPath(paths.upperBalls);
                    setPathState(Paths.PathState.upperEat);
                    break;

                case upperEat:
                    Intake();
                    follower.followPath(paths.upperEat);
                    setPathState(Paths.PathState.upperTurn);
                    break;

                case upperTurn:
                    follower.followPath(paths.upperTurn);
                    setPathState(Paths.PathState.toShoot);
                    break;

                case toShoot:
                    Shoot();
                    follower.followPath(paths.toShoot);
                    setPathState(Paths.PathState.finalPose);
                    break;

                // --- End / park / random ---
                case finalPose:
                    follower.followPath(paths.finalPose, true); // true if you want hold/end

                    break;

            }
        }
    }


    public static class Paths
    {
        public Pose startPose = new Pose(21.000, 125.000, Math.toRadians(39));

        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
        // Gate-specific paths used in buildPathsGate
        Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, finalPose;

        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
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
            startPose = new Pose(21.000, 125.000,  Math.toRadians(145));
            final Pose shootPose = new Pose(43.000, 108.000);
            final Pose leavePose = new Pose(19.000, 106.000);

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

            startPose = new Pose(20.000, 123.000);
            final Pose ballOneLinePose = new Pose(54.000, 84.000);
            final Pose ballOneFullPose = new Pose(18.000, 84.000);
            final Pose shootPose = new Pose(54.000, 97.200);
            final Pose ballTwoLinePose = new Pose(54.000, 60.000);
            final Pose ballTwoFullPose = new Pose(17.000, 60.000);
            final Pose ballThreeLinePose = new Pose(54.000, 35.000);
            final Pose ballThreeFullPose = new Pose(16.000, 35.000);
            final Pose randomPose = new Pose(53.890, 69.156);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,
                                    shootPose

                            )).setTangentHeadingInterpolation()
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

            ToBallTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoLinePose,

                                    ballTwoFullPose))
                    .setTangentHeadingInterpolation()

                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoFullPose,

                                    shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            ToThree = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballThreeLinePose

                            )).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

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

            finalPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    randomPose

                            )).setConstantHeadingInterpolation(Math.toRadians(143))

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