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

@Autonomous(name = "CloseBlueAuto", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends Base {


    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;
    private AutoStrat autoStrat = AutoStrat.REGULAR;
    private Paths.PathState currentPathState;

    @Override
    public void runOpMode() throws InterruptedException {
        team = Team.BLUE;
        super.runOpMode();

        if (gamepad1.dpad_up) {
            autoStrat = AutoStrat.GATE;
        } else if (gamepad1.dpad_right) {
            autoStrat = AutoStrat.REGULAR;
        } else if (gamepad1.dpad_down) {
            autoStrat = AutoStrat.BASIC;
        }

        telemetry.addLine("--- SELECT AUTO STRATEGY ---");
        telemetry.addData("Selected", autoStrat);
        telemetry.addLine("\nControls:");
        telemetry.addLine("DPAD UP: 12 Ball");
        telemetry.addLine("DPAD RIGHT: 9 Ball");
        telemetry.addLine("DPAD DOWN: 3 Ball");
        telemetry.update();

        initAuto();

        waitForStart();
        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {
            opModeTimer.resetTimer();

            if(autoStrat == AutoStrat.BASIC) {
                // Basic Start
            }else if (autoStrat == AutoStrat.REGULAR) {

                follower.followPath(paths.ToShoot);

            }else{
                // Gate Start
            }

            while (opModeIsActive() && !isStopRequested()) {
                handlePathing();
                follower.update();

                telemetry.addData("Current State", currentPathState);
                telemetry.addData("State Time (s)", timer.getElapsedTimeSeconds());
                telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
                telemetry.update();
            }
        }
    }

    public void initAuto() {
        follower = PedroConstants.createFollower(hardwareMap);
        paths = new Paths(follower, autoStrat);
        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        if(autoStrat == AutoStrat.BASIC) {
            // Basic Start
        }else if (autoStrat == AutoStrat.REGULAR) {

            setPathState(Paths.PathState.ToShoot);

        }else{
            // Gate Start
        }


    }

    public void setPathState(Paths.PathState pathState) {
        currentPathState = pathState;
        timer.resetTimer();
    }

    private void handlePathing() {
        switch (autoStrat){

               case BASIC:
                   PathBasic();
                    break;
                case GATE:
                    PathRegular();
                    break;
                case REGULAR:
                default:
                    PathGath();
                    break;
            }

    }

    private void PathBasic() {
        // This check ensures we only try to start a new path *after* the current one is complete.
        if (!follower.isBusy()) {
            switch (currentPathState) {
                // The logic is now: when a state is finished, start the NEXT path and set the NEXT state.
                case ToShoot:
                    // shoot();
                    follower.followPath(paths.ToBallOne); // Start NEXT path
                    setPathState(Paths.PathState.ToBallOne); // Set NEXT state
                    break;
                case ToBallOne:
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;
                case ToBallOneFull:
                    // intake();
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;
                case ToShoot_1:
                    // shoot();
                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;
                case ToBallTwo:
                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;
                case ToBallTwoFull:
                    // intake();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;
                case ToShoot_2:
                    // shoot();
                    follower.followPath(paths.Gate1);
                    setPathState(Paths.PathState.Gate1);
                    break;
                case Gate1:
                    follower.followPath(paths.Gate2);
                    setPathState(Paths.PathState.Gate2);
                    break;
                case Gate2:
                    follower.followPath(paths.ToEatGate);
                    setPathState(Paths.PathState.ToEatGate);
                    break;
                case ToEatGate:
                    // intake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;
                case ToShoot_3:
                    // shoot();
                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;
                case ToThree:
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;
                case ToThreeFull:
                    // intake();
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;
                case ToShoot_4:
                    // shoot();
                    follower.followPath(paths.ToRandom, true);
                    setPathState(Paths.PathState.ToRandom);
                    break;
                case ToRandom:
                    // Autonomous is complete. Do nothing.
                    break;
            }
        }
    }
    private void PathRegular() {
        // This check ensures we only try to start a new path *after* the current one is complete.
        if (!follower.isBusy()) {
            switch (currentPathState) {
                // The logic is now: when a state is finished, start the NEXT path and set the NEXT state.
                case ToShoot:
                    // shoot();
                    follower.followPath(paths.ToBallOne); // Start NEXT path
                    setPathState(Paths.PathState.ToBallOne); // Set NEXT state
                    break;
                case ToBallOne:
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;
                case ToBallOneFull:
                    // intake();
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;
                case ToShoot_1:
                    // shoot();
                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;
                case ToBallTwo:
                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;
                case ToBallTwoFull:
                    // intake();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;
                case ToShoot_2:
                    // shoot();
                    follower.followPath(paths.Gate1);
                    setPathState(Paths.PathState.Gate1);
                    break;
                case Gate1:
                    follower.followPath(paths.Gate2);
                    setPathState(Paths.PathState.Gate2);
                    break;
                case Gate2:
                    follower.followPath(paths.ToEatGate);
                    setPathState(Paths.PathState.ToEatGate);
                    break;
                case ToEatGate:
                    // intake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;
                case ToShoot_3:
                    // shoot();
                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;
                case ToThree:
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;
                case ToThreeFull:
                    // intake();
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;
                case ToShoot_4:
                    // shoot();
                    follower.followPath(paths.ToRandom, true);
                    setPathState(Paths.PathState.ToRandom);
                    break;
                case ToRandom:
                    // Autonomous is complete. Do nothing.
                    break;
            }
        }
    }
    private void PathGath() {
        // This check ensures we only try to start a new path *after* the current one is complete.
        if (!follower.isBusy()) {
            switch (currentPathState) {
                // The logic is now: when a state is finished, start the NEXT path and set the NEXT state.
                case ToShoot:
                    // shoot();
                    follower.followPath(paths.ToBallOne); // Start NEXT path
                    setPathState(Paths.PathState.ToBallOne); // Set NEXT state
                    break;
                case ToBallOne:
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                    break;
                case ToBallOneFull:
                    // intake();
                    follower.followPath(paths.ToShoot_1);
                    setPathState(Paths.PathState.ToShoot_1);
                    break;
                case ToShoot_1:
                    // shoot();
                    follower.followPath(paths.ToBallTwo);
                    setPathState(Paths.PathState.ToBallTwo);
                    break;
                case ToBallTwo:
                    follower.followPath(paths.ToBallTwoFull);
                    setPathState(Paths.PathState.ToBallTwoFull);
                    break;
                case ToBallTwoFull:
                    // intake();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;
                case ToShoot_2:
                    // shoot();
                    follower.followPath(paths.Gate1);
                    setPathState(Paths.PathState.Gate1);
                    break;
                case Gate1:
                    follower.followPath(paths.Gate2);
                    setPathState(Paths.PathState.Gate2);
                    break;
                case Gate2:
                    follower.followPath(paths.ToEatGate);
                    setPathState(Paths.PathState.ToEatGate);
                    break;
                case ToEatGate:
                    // intake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;
                case ToShoot_3:
                    // shoot();
                    follower.followPath(paths.ToThree);
                    setPathState(Paths.PathState.ToThree);
                    break;
                case ToThree:
                    follower.followPath(paths.ToThreeFull);
                    setPathState(Paths.PathState.ToThreeFull);
                    break;
                case ToThreeFull:
                    // intake();
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;
                case ToShoot_4:
                    // shoot();
                    follower.followPath(paths.ToRandom, true);
                    setPathState(Paths.PathState.ToRandom);
                    break;
                case ToRandom:
                    // Autonomous is complete. Do nothing.
                    break;
            }
        }
    }



    public static class Paths {

        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree,
                ToThreeFull, ToRandom, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
        // Gate-specific paths used in buildPathsGate
        Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
                bottomBalls, bottomBallsEat, ToShoot_5, upperBalls, upperEat, upperTurn, toShoot, finalPose;

        public enum PathState {
            ToShoot, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree,
            ToThreeFull, ToRandom, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
            // Gate-specific states
            Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3,
            bottomBalls, bottomBallsEat, ToShoot_5, Path17, upperBalls, upperTurn, toShoot, finalPose
        }

        public Paths(Follower follower, AutoStrat strat) {

            switch (strat) {
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

        private void buildPathsBasic(Follower follower) {
            final Pose startPose = new Pose(20.000, 123.000);
            final Pose shootPose = new Pose(54.000, 97.200);
            final Pose ballOneLinePose = new Pose(54.000, 84.000);
            final Pose ballOneFullPose = new Pose(19.000, 84.000);
            final Pose ballTwoLinePose = new Pose(54.000, 58.000);
            final Pose ballTwoFullPose = new Pose(19.000, 58.000);
            final Pose gateLinePose = new Pose(24.000, 69.000);
            final Pose gatePushPose = new Pose(16.000, 68.000);
            final Pose gateEatPose = new Pose(9.000, 58.000);
            final Pose ballThreePose = new Pose(54.000, 35.000);
            final Pose ballThreeFullPose = new Pose(19.000, 35.000);
            final Pose RandomPose = new Pose(54.000, 76.000);

            ToShoot = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToBallOne = follower.pathBuilder().addPath(new BezierLine(shootPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(143))
                    .build();

            ToBallTwo = follower.pathBuilder().addPath(new BezierLine(shootPose, ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(new BezierLine(ballTwoLinePose, ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(new BezierLine(ballTwoFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            Gate1 = follower.pathBuilder().addPath(new BezierLine(shootPose, gateLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Gate2 = follower.pathBuilder().addPath(new BezierLine(gateLinePose, gatePushPose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ToEatGate = follower.pathBuilder().addPath(new BezierLine(gatePushPose, gateEatPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                    .build();

            ToShoot_3 = follower.pathBuilder().addPath(new BezierLine(gateEatPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(143))
                    .build();

            ToThree = follower.pathBuilder().addPath(new BezierLine(shootPose, ballThreePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToThreeFull = follower.pathBuilder().addPath(new BezierLine(ballThreePose, ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_4 = follower.pathBuilder().addPath(new BezierLine(ballThreeFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(143))
                    .build();

            ToRandom = follower.pathBuilder().addPath(new BezierLine(shootPose, RandomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(113))
                    .build();
        }
        private void buildPathsReg(Follower follower) {
            final Pose startPose = new Pose(20.000, 123.000);
            final Pose shootPose = new Pose(54.000, 97.200);
            final Pose ballOneLinePose = new Pose(54.000, 84.000);
            final Pose ballOneFullPose = new Pose(19.000, 84.000);
            final Pose ballTwoLinePose = new Pose(54.000, 58.000);
            final Pose ballTwoFullPose = new Pose(19.000, 58.000);
            final Pose gateLinePose = new Pose(24.000, 69.000);
            final Pose gatePushPose = new Pose(16.000, 68.000);
            final Pose gateEatPose = new Pose(9.000, 58.000);
            final Pose ballThreePose = new Pose(54.000, 35.000);
            final Pose ballThreeFullPose = new Pose(19.000, 35.000);
            final Pose RandomPose = new Pose(54.000, 76.000);

            ToShoot = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToBallOne = follower.pathBuilder().addPath(new BezierLine(shootPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(143))
                    .build();

            ToBallTwo = follower.pathBuilder().addPath(new BezierLine(shootPose, ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(new BezierLine(ballTwoLinePose, ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(new BezierLine(ballTwoFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();

            Gate1 = follower.pathBuilder().addPath(new BezierLine(shootPose, gateLinePose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Gate2 = follower.pathBuilder().addPath(new BezierLine(gateLinePose, gatePushPose))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ToEatGate = follower.pathBuilder().addPath(new BezierLine(gatePushPose, gateEatPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                    .build();

            ToShoot_3 = follower.pathBuilder().addPath(new BezierLine(gateEatPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(143))
                    .build();

            ToThree = follower.pathBuilder().addPath(new BezierLine(shootPose, ballThreePose))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            ToThreeFull = follower.pathBuilder().addPath(new BezierLine(ballThreePose, ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_4 = follower.pathBuilder().addPath(new BezierLine(ballThreeFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(143))
                    .build();

            ToRandom = follower.pathBuilder().addPath(new BezierLine(shootPose, RandomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(113))
                    .build();
        }
        private void buildPathsGate(Follower follower) {
            final Pose startPose = new Pose(67.000, 8.000);
            final Pose shootPose =  new Pose(71.000, 71.000);
            final Pose ballOneLinePose =  new Pose(56.000, 60.000);
            final Pose ballOneFullPose = new Pose(56.000, 60.000);
            final Pose gateLinePose =  new Pose(15.000, 71.000);
            final Pose gateControlPoint = new Pose(20.000, 64.000);
            final Pose eatLinePose = new Pose(10.000, 61.000);
            final Pose ballThreePose = new Pose(71.000, 35.000);
            final Pose ballThreeFullPose = new Pose(15.000, 35.000);
            final Pose ballPickPose = new Pose(58.000, 84.000);
            final Pose TurnPose =  new Pose(50.000, 84.000);
            final Pose topEatPose = new Pose(16.000, 84.000);
            final Pose randomPose = new Pose(28.000, 84.000);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose ,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(144))

                    .build();

            ToBallOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballOneLinePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneLinePose,

                                    ballOneFullPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballOneFullPose,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    gateLinePose

                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Eat = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateLinePose,

                                    gateControlPoint,

                                    eatLinePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    eatLinePose,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))

                    .build();

            Gate_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    gateLinePose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Eat_2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateLinePose,

                                    gateControlPoint,

                                    eatLinePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

                    .build();

            ToShoot_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    eatLinePose,

                                   shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))

                    .build();

            Gate_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    gateLinePose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Eat_3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    gateLinePose,
                                    gateControlPoint,
                                    eatLinePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

                    .build();

            ToShoot_4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    eatLinePose,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))

                    .build();

            bottomBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballThreePose

                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                    .build();

            bottomBallsEat = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreePose,

                                    ballThreeFullPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToShoot_5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeFullPose,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            upperBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballPickPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            upperEat = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballPickPose,

                                    TurnPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            upperTurn = follower.pathBuilder().addPath(
                            new BezierLine(
                                    TurnPose,

                                    topEatPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            toShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    topEatPose,

                                    TurnPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                    .build();

            finalPose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    TurnPose,

                                    randomPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }


    }

}