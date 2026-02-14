package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close.CompClose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;

@Autonomous(name = "ShootingCloseRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class RedComp extends AutoBase
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
                autoStrat = AutoStrat.BASIC;
            } else if (gamepad1.dpad_right)
            {
                autoStrat = AutoStrat.REGULAR;
            } else if (gamepad1.dpad_down)
            {
                autoStrat = AutoStrat.REGULAR;
            }

            // --- Keep your telemetry inside the loop for live feedback ---
            telemetry.addLine("--- SELECT AUTO STRATEGY ---");
            telemetry.addData("Selected", autoStrat);
            telemetry.addLine("\nControls:");

            telemetry.addLine("DPAD UP: 3 Ball (BASIC)");

            telemetry.addLine("DPAD RIGHT: 9 Ball (REGULAR)");

            telemetry.update();
        }

        // Now the selection is locked in. Initialize paths and timers.
        initAuto();

        // The OpMode will wait here until you press START
        waitForStart();

        if (opModeIsActive() && !isStopRequested())
        {
            opModeTimer.resetTimer();
            startOuttake();
            // follower.followPath(paths.ToShoot);

            while (opModeIsActive() && !isStopRequested() && !finishedAutonomous)
            {
                handlePathing();
                follower.update();

                updateSubsystems();
                displayTelemetry();
            }
        }
    }

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

    public void initAuto()
    {
        follower = PedroConstants.createFollower(hardwareMap);
        setFollower(follower);
        paths = new Paths(follower,  autoStrat);
        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower.setStartingPose(paths.startPose);
        follower.update();

        initRobot();

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
        if (!follower.isBusy()) {
            switch (currentPathState) {

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

    private void PathRegular()
    {
        // This check ensures we only try to start a new path *after* the current one is
        // complete.
        if (!follower.isBusy())
        {
            switch (currentPathState)
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

    private void PathGate()
    {

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

        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4,
            // Gate-specific states
            Gate, Eat, Gate_2, Eat_2, Gate_3, Eat_3, bottomBalls, bottomBallsEat, ToShoot_5, upperEat, upperBalls, upperTurn, toShoot, finalPose, FinalPos
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
            // --- Pose definitions ---
            startPose = new Pose(79, 9, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(136, 26);
            final Pose ballOneFullPose = new Pose(136, 9);

            final Pose shootPose = new Pose(79, 9);

            final Pose randomPose = new Pose(120, 12);

            // --- Paths ---
            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270))
                    .build();

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();


            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, randomPose))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        private void buildPathsReg(Follower follower)
        {
            // --- Pose definitions ---
            startPose = new Pose(84, 9, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(84, 36);
            final Pose ballOneFullPose = new Pose(125, 36);

            final Pose shootPose = new Pose(84, 9);

            final Pose randomPose = new Pose(120, 12);

            // --- Paths ---
            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setTangentHeadingInterpolation()
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


        private void buildPathsGate(Follower follower) {

        }
    }
}