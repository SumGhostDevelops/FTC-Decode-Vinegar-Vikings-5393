package org.firstinspires.ftc.teamcode.opmodes.autonomous.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;


@Autonomous(name = "FarRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
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
                        }
                        else if (gamepad1.dpad_right)
                        {
                                autoStrat = AutoStrat.REGULAR;
                        } else if (gamepad1.dpad_down)
                        {
                                autoStrat = AutoStrat.BASIC;
                        }

                        // CUSTOM AND LEGAL TELEMETRY MENU
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

                            RobotUpdates();
                        }
                }
        }
    @Override
    protected void displayTelemetry()
    {
        telemetry.addData("Current State", currentPathState);
        telemetry.addData("State Time (s)", timer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addLine("-----");
        telemetry.addData("Distance to Goal", getPose2d().distanceTo(getGoal()).toUnit(DistanceUnit.INCH));
        telemetry.addData("Outtake RPM", robot.subsystems.outtake.getMotorRPM());
        telemetry.addData("Turret Angle", robot.subsystems.turret.getRelativeAngle().toUnit(AngleUnit.DEGREES));
        telemetry.addData("Turret Target Angle", robot.subsystems.turret.getTargetAngleDegrees());
        telemetry.update();
    }
            /**
             * --- Initialize robot and paths ---
             * Starts Local Timer, and Sets Follower Pose
             * Starts First Path.
             */
        public void initAuto()
        {

                // --- Initialize pedro Path Constants and Followers ---
                follower = PedroConstants.createFollower(hardwareMap);
                setFollower(follower);
                paths = new Paths(follower, autoStrat);
                // --- Initialize Timers ---
                timer = new Timer();
                opModeTimer = new Timer();
                opModeTimer.resetTimer();
                //Sets Starting pose and updates follower
                follower.setStartingPose(paths.startPose);
                follower.update();
                //Init Robot Hardware.
                initRobot();
                //starts first Path
                setPathState(Paths.PathState.ToShoot);



        }



        private void handlePathing()
        {
            //Sets paths based on set strat
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
    public void setPathState(Paths.PathState pathState)
    {
        currentPathState = pathState;
        timer.resetTimer();
    }
        private void PathBasic()
        {

        if (!follower.isBusy())
     {
            switch (currentPathState)
            {
                    case ToShoot:


                            follower.followPath(paths.Move);
                            setPathState(Paths.PathState.Move);

                            break;
                    case Move:
                        // after it reachest the last state, it starts moving.
                        Shoot();

                        break;
            }
        }
    }

    private void PathRegular() {
        if (!follower.isBusy())
        {
            switch (currentPathState)
            {
                //  When in a state, start the NEXT path.
                case ToShoot:
                    // before following next path, it shoots.
                follower.followPath(paths.ToShoot);
                  setPathState(Paths.PathState.ToShootFar);

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
                    setPathState(Paths.PathState.ToThree); // Corrected this from Gate1
                    break;
                case ToThree:
                    startIntake();
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull); // Corrected this from Gate2
                    break;
                case ToThreeFull:
                    stopIntake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3); // Corrected this from ToEatGate
                    break;
                case ToShoot_3:
                    Shoot();

                    follower.followPath(paths.FinalPose);
                    // Set the state to match the path you just started
                    setPathState(Paths.PathState.finalPose);
                    break;
                case finalPose:

                    break;
            }
        }
    }


    private void PathGate() {
        if (!follower.isBusy()) {
            switch (currentPathState) {
                // Corrected Logic: When a state is finished, start the NEXT path and set the NEXT state.
                case ToShoot:
                    Shoot();
                    follower.followPath(paths.ToBallOne); // Start path TO BallOne
                    setPathState(Paths.PathState.ToBallOne);
                    break;
                case ToBallOne:
                    
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;
                case ToBallOneFull:
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;
                case ToShoot_1:
                    Shoot();
                    follower.followPath(paths.Gate);
                    setPathState(Paths.PathState.Gate);
                    break;
                case Gate:
                    follower.followPath(paths.Eat);
                    setPathState(Paths.PathState.Eat);
                    break;
                case Eat:
                    
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;
                case ToShoot_2:
                    Shoot();
                    follower.followPath(paths.Gate_2);
                    setPathState(Paths.PathState.Gate_2);
                    break;
                case Gate_2:
                    follower.followPath(paths.Eat_2);
                    setPathState(Paths.PathState.Eat_2);
                    break;
                case Eat_2:
                    
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;
                case ToShoot_3:
                    Shoot();
                    follower.followPath(paths.Gate_3);
                    setPathState(Paths.PathState.Gate_3);
                    break;
                case Gate_3:
                    follower.followPath(paths.Eat_3);
                    setPathState(Paths.PathState.Eat_3);
                    break;
                case Eat_3:
                    
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;
                case ToShoot_4:
                    Shoot();
                    follower.followPath(paths.bottomBalls);
                    setPathState(Paths.PathState.bottomBalls);
                    break;
                case bottomBalls:
                    follower.followPath(paths.bottomBallsEat);
                    setPathState(Paths.PathState.bottomBallsEat);
                    break;
                case bottomBallsEat:
                    
                    follower.followPath(paths.ToShoot_5);
                    setPathState(Paths.PathState.ToShoot_5);
                    break;
                case ToShoot_5:
                    Shoot();
                    follower.followPath(paths.upperBalls);
                    setPathState(Paths.PathState.upperBalls);
                    break;
                case upperBalls:
                    follower.followPath(paths.upperEat);
                    setPathState(Paths.PathState.upperEat);
                    break;
                case upperEat:
                    
                    follower.followPath(paths.upperTurn);
                    setPathState(Paths.PathState.upperTurn);
                    break;
                case upperTurn:
                    follower.followPath(paths.toShoot);
                    setPathState(Paths.PathState.toShoot);
                    break;
                case toShoot:
                    Shoot();
                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.finalPose);
                    break;
                case finalPose:
                    follower.followPath(paths.finalPose, true);
                    break;
            }
        }
    }

        public static class Paths
        {
                public Pose startPose = new Pose(123, 125, Math.toRadians(90));

                public PathChain ToShoot,
                                ToBallOne, ToBallOneFull, ToShootFar, ToBallTwo, ToBallTwoFull, ToThree,
                                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
                                // Gate-specific paths used in buildPathsGate
                                Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, Move,
                                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, FinalPose, finalPose;

                public enum PathState
                {
                        ToShoot, ToBallOne,  ToShootFar, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4, Move,
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
                        startPose = new Pose(123, 125, Math.toRadians(38));
                        final Pose shootPose = new Pose(101, 108);
                        final Pose randomPose = new Pose(125, 106);

                        ToShoot = follower.pathBuilder()
                                        .addPath(new BezierLine(startPose, shootPose))
                                        .setTangentHeadingInterpolation()
                                        .setReversed()
                                        .build();
                        Move = follower.pathBuilder()
                                        .addPath(new BezierLine(shootPose, randomPose))
                                        .setConstantHeadingInterpolation(Math.toRadians(38))
                                        .build();

                }

                private void buildPathsReg(Follower follower)
                {
// --- Pose definitions ---
                    startPose = new Pose(124, 123, Math.toRadians(37));

                    final Pose shootPose = new Pose(90.0, 97.200);
                    final Pose ballOneLinePose = new Pose(90.0, 84);
                    final Pose ballOneFullPose = new Pose(125, 84);

                    final Pose gateLinePose = new Pose(90.0, 58);
                    final Pose gatePushPose = new Pose(130, 58);

                    final Pose ballThreeLinePose = new Pose(90.0, 35);
                    final Pose ballThreePose = new Pose(135.0, 35);

                    final Pose finalPose = new Pose(118.0, 96.0);

// --- Paths ---
                    ToShoot = follower.pathBuilder()
                            .addPath(new BezierLine(startPose, shootPose))
                            .setTangentHeadingInterpolation()
                            .setReversed()
                            .build();

                    ToBallOne = follower.pathBuilder()
                            .addPath(new BezierLine(shootPose, ballOneLinePose))
                            .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                            .build();

                    ToBallOneFull = follower.pathBuilder()
                            .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                            .setTangentHeadingInterpolation()
                            .build();

                    ToShoot_1 = follower.pathBuilder()
                            .addPath(new BezierLine(ballOneFullPose, shootPose))
                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                            .build();

                    ToBallTwo = follower.pathBuilder()
                            .addPath(new BezierLine(shootPose, gateLinePose))
                            .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                            .build();

                    ToBallTwoFull = follower.pathBuilder()
                            .addPath(new BezierLine(gateLinePose, gatePushPose))
                            .setTangentHeadingInterpolation()
                            .build();

                    ToShoot_2 = follower.pathBuilder()
                            .addPath(new BezierLine(gatePushPose, shootPose))
                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                            .build();

                    ToThree = follower.pathBuilder()
                            .addPath(new BezierLine(shootPose, ballThreeLinePose))
                            .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                            .build();

                    ToThreeFull = follower.pathBuilder()
                            .addPath(new BezierLine(ballThreeLinePose, ballThreePose))
                            .setTangentHeadingInterpolation()
                            .build();

                    ToShoot_3 = follower.pathBuilder()
                            .addPath(new BezierLine(ballThreePose, shootPose))
                            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                            .build();

                    FinalPose = follower.pathBuilder()
                            .addPath(new BezierLine(shootPose, finalPose))
                            .setConstantHeadingInterpolation(Math.toRadians(37))
                            .build();

                }

                private void buildPathsGate(Follower follower)
                {
                    startPose = new Pose(123, 125, Math.toRadians(38));
                        final Pose shootPose = new Pose(73, 71);
                        final Pose ballOneLinePose = new Pose(88.000, 60.000);
                        final Pose ballOneFullPose = new Pose(129.000, 60.000);
                        final Pose gateLinePose = new Pose(127, 64);
                        final Pose gateControlPoint = new Pose(124, 62);
                        final Pose eatLinePose = new Pose(134.000, 61.000);
                        final Pose ballThreePose = new Pose(71.000, 35.000);
                        final Pose ballThreeFullPose = new Pose(129.000, 35.000);
                        final Pose ballPickPose = new Pose(86.000, 84.000);
                        final Pose TurnPose = new Pose(128.000, 84.000);
                        final Pose topEatPose = new Pose(86.000, 84.000);
                        final Pose randomPose = new Pose(116.000, 84.000);

                        ToShoot = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        startPose,

                                                        shootPose))
                                        .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(38))

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