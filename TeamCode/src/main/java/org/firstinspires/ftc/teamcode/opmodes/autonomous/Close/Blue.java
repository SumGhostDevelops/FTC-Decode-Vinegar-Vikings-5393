package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;

import java.util.Objects;

@Autonomous(name = "CloseBlueAuto", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends AutoBase
{

        private Paths paths;
        private Timer timer, opModeTimer;
        private Follower follower;
        private AutoStrat autoStrat = AutoStrat.REGULAR;
        private Paths.PathState currentPathState;

        @Override
        public void runOpMode() throws InterruptedException
        {
                team = Team.BLUE;
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

                        while (opModeIsActive() && !isStopRequested())
                        {
                                handlePathing();
                                follower.update();

                                updateSubsystems();

                                telemetry.addData("Current State", currentPathState);
                                telemetry.addData("State Time (s)", timer.getElapsedTimeSeconds());
                                telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
                                telemetry.addData("Heading", follower.getHeading());
                                telemetry.update();
                        }
                }
        }

    public void initAuto()
    {


                follower = PedroConstants.createFollower(hardwareMap);
                setFollower(follower);
                paths = new Paths(follower, autoStrat);
                timer = new Timer();
                opModeTimer = new Timer();
                opModeTimer.resetTimer();

                follower.setStartingPose(paths.startPose);
                follower.update();
                initRobot();
                setPathState(Paths.PathState.ToShoot);
    }



    public void Intake()
    {
        if (robot == null)
            return;

        closeTransfer();
        startIntake();
    }

    public void Shoot()
    {
        if (robot == null)
            return;

        startOuttake();
        closeTransfer();

        waitForSystemsReady(1500);

        if (isReadyToShoot())
        {
            shootForDuration(500);
        }
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
                                        break;
                                case FinalPose:
                                        // end path
                                        follower.followPath(paths.FinalPose);
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
                                    // pause to build,
                                    // shoot
                                        Shoot();
                                        follower.followPath(paths.ToBallOne); // Start NEXT path
                                    // turn on intake
                                        setPathState(Paths.PathState.ToBallOne); // Set NEXT state
                                        break;
                                case ToBallOne:
                                        Intake();
                                        follower.followPath(paths.ToBallOneFull);
                                        setPathState(Paths.PathState.ToBallOneFull);
                                        break;
                                case ToBallOneFull:
                                        // has ball
                                    stopIntake();
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
                                        Intake();
                                        follower.followPath(paths.ToThreeFull);
                                        setPathState(Paths.PathState.ToThreeFull);
                                        break;
                                case ToThreeFull:
                                        // has ball
                                    stopIntake();
                                    follower.followPath(paths.bottomBalls);
                                        setPathState(Paths.PathState.bottomBalls);
                                        break;
                                case bottomBalls:
                                        Intake();
                                        follower.followPath(paths.bottomBallsEat);
                                        setPathState(Paths.PathState.bottomBallsEat);
                                        break;
                                case bottomBallsEat:
                                        Shoot();
                                        break;
                        }
                }
        }

    private void PathGate() {
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
        if (!follower.isBusy()) {
            switch (currentPathState) {

                // The logic is: when a state is finished, start the NEXT path and set the NEXT state.

                // --- Start: go shoot first ---
                case ToShoot:
                    Shoot();
                    follower.followPath(paths.ToBallOne); // Start path to BallOne
                    setPathState(Paths.PathState.ToBallOne); // Set state to BallOne
                    break;

                // --- Ball 1 sequence ---
                case ToBallOne:
                    Intake();
                    follower.followPath(paths.ToBallOneFull); // Start path to BallOneFull
                    setPathState(Paths.PathState.ToBallOneFull); // Set state to BallOneFull
                    break;

                case ToBallOneFull:
                    // has ball
                    stopIntake();
                    follower.followPath(paths.ToShoot_1); // Start path to Shoot_1
                    setPathState(Paths.PathState.ToShoot_1); // Set state to Shoot_1
                    break;

                case ToShoot_1:
                    Shoot();
                    follower.followPath(paths.Gate); // Start path to Gate
                    setPathState(Paths.PathState.Gate); // Set state to Gate
                    break;

                // --- Gate cycle 1 ---
                case Gate:
                    follower.followPath(paths.Eat); // Start path to Eat
                    setPathState(Paths.PathState.Eat); // Set state to Eat
                    break;

                case Eat:
                    Intake(); // Assuming you intake here
                    follower.followPath(paths.ToShoot_2); // Start path to Shoot_2
                    setPathState(Paths.PathState.ToShoot_2); // Set state to Shoot_2
                    break;

                case ToShoot_2:
                    stopIntake();
                    Shoot();
                    follower.followPath(paths.Gate_2); // Start path to Gate_2
                    setPathState(Paths.PathState.Gate_2); // Set state to Gate_2
                    break;

                // --- Gate cycle 2 ---
                case Gate_2:
                    follower.followPath(paths.Eat_2); // Start path to Eat_2
                    setPathState(Paths.PathState.Eat_2); // Set state to Eat_2
                    break;

                case Eat_2:
                    Intake();
                    follower.followPath(paths.ToShoot_3); // Start path to Shoot_3
                    setPathState(Paths.PathState.ToShoot_3); // Set state to Shoot_3
                    break;

                case ToShoot_3:
                    stopIntake();
                    Shoot();
                    follower.followPath(paths.Gate_3); // Start path to Gate_3
                    setPathState(Paths.PathState.Gate_3); // Set state to Gate_3
                    break;

                // --- Gate cycle 3 ---
                case Gate_3:
                    follower.followPath(paths.Eat_3); // Start path to Eat_3
                    setPathState(Paths.PathState.Eat_3); // Set state to Eat_3
                    break;

                case Eat_3:
                    Intake();
                    follower.followPath(paths.ToShoot_4); // Start path to Shoot_4
                    setPathState(Paths.PathState.ToShoot_4); // Set state to Shoot_4
                    break;

                case ToShoot_4:
                    stopIntake();
                    Shoot();
                    // This is the last action, so now we can go to the final parking position.
                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.finalPose);
                    break;

                case finalPose:
                    // The autonomous routine is complete.
                    // You can hold the final position or simply do nothing.
                    break;
            }
        }
    }

    public static class Paths
        {

                public Pose startPose = new Pose(123, 125, Math.toRadians(90));
                public PathChain ToShoot,
                                ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, ToThree,
                                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
                                // Gate-specific paths used in buildPathsGate
                                Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
                                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, FinalPose, finalPose;

                public enum PathState
                {
                        ToShoot, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
                        // Gate-specific states
                        Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, bottomBalls, bottomBallsEat, ToShoot_5, upperEat, upperBalls, upperTurn, toShoot, finalPose, FinalPose
                }

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
                        final Pose shootPose = new Pose(80, 27); // adjust values if distance is incorrect
                        final Pose finalPose = new Pose(80, 38.5);

                        ToShoot = follower.pathBuilder()
                                        .addPath(new BezierLine(startPose, shootPose))
                                        .setTangentHeadingInterpolation()
                                        .build();

                        FinalPose = follower.pathBuilder()
                                        .addPath(new BezierLine(shootPose, finalPose))
                                        .setConstantHeadingInterpolation(Math.toRadians(142))
                                        .build();

                }

                private void buildPathsReg(Follower follower)
                {

                        // --- Pose definitions ---
                        startPose = new Pose(65.000, 9.000, Math.toRadians(90));
                        final Pose ballOneLinePose = new Pose(40.000, 36.000);
                        final Pose ballOneFullPose = new Pose(19.000, 36.000);
                        final Pose shootPose = new Pose(69.000, 68.000);
                        final Pose ballTwoLinePose = new Pose(41.000, 60.000);
                        final Pose ballTwoFullPose = new Pose(19.000, 60.000);
                        final Pose ballThreeLinePose = new Pose(40.000, 84.000);
                        final Pose ballThreeFullPose = new Pose(19.000, 84.000);
                        final Pose gateLinePose = new Pose(54.000, 84.000);
                        final Pose gatePushPose = new Pose(49.000, 79.000);

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

                        ToShoot_1 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        ballOneFullPose,
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

                        ToShoot_2 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        ballTwoFullPose,
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
                                                        gateLinePose))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                                        .build();

                        bottomBallsEat = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        gateLinePose,
                                                        gatePushPose))
                                        .setConstantHeadingInterpolation(Math.toRadians(136))
                                        .build();
                }

                private void buildPathsGate(Follower follower)
                {
                        startPose = new Pose(67.000, 8.000, Math.toRadians(90));
                        final Pose shootPose = new Pose(71.000, 71.000);
                        final Pose ballOneLinePose = new Pose(56.000, 60.000);
                        final Pose ballOneFullPose = new Pose(15.000, 60.000);
                        final Pose gateLinePose = new Pose(17, 66);
                        final Pose gateControlPoint = new Pose(20.000, 64.000);
                        final Pose eatLinePose = new Pose(10.000, 59.000);
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
                                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(144))

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

                                                        gateLinePose

                                        )).setTangentHeadingInterpolation()

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

                                                        ballThreePose

                                        )).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

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
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

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

                                                        TurnPose))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                                        .build();

                        finalPose = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        TurnPose,

                                                        randomPose))
                                        .setTangentHeadingInterpolation()

                                        .build();
                }

        }

}