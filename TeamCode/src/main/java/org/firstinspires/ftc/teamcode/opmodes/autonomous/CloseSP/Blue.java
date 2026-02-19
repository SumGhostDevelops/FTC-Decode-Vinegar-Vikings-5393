package org.firstinspires.ftc.teamcode.opmodes.autonomous.CloseSP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;


@Autonomous(name = "SpecialCloseBlueAuto", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends AutoBase
{

    private Paths paths;

    @Override
    protected StrategyOption[] getStrategyOptions()
    {
        return new StrategyOption[]{
                new StrategyOption(AutoStrat.REGULAR, "DPAD RIGHT", "12 Ball Close (REGULAR)"),
                new StrategyOption(AutoStrat.BASIC, "DPAD DOWN", "6 Ball (BASIC)")
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

    private void setPathState(Paths.PathState pathState)
    {
        setPathState((Object) pathState);
    }

    private Paths.PathState getPathState()
    {
        return (Paths.PathState) currentPathState;
    }

    @Override
    protected void displayTelemetry()
    {
        telemetry.addData("Current State", currentPathState);
        telemetry.addData("State Time (s)", pathStateTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addLine("-----");
        telemetry.addData("Pose", getPose2d());
        telemetry.addData("Velocity", follower.getVelocity());
        telemetry.addData("Distance to Goal", getPose2d().distanceTo(getGoal()).toUnit(DistanceUnit.INCH));
        telemetry.addData("Outtake RPM", subsystems.outtake.getMotorRPM());
        telemetry.addData("Turret Angle", subsystems.turret.getRelativeAngle().toUnit(AngleUnit.DEGREES));
        telemetry.addData("Turret Target Angle", subsystems.turret.getTargetAngleDegrees());
        telemetry.update();
    }

    @Override
    protected void PathBasic()
    {
        if (!follower.isBusy())
        {
            switch (getPathState())
            {
                case ToShoot:
                    // Shoots first, then drives to final pose
                    Shoot();
                    follower.followPath(paths.FinalPose, false);
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

                case ToBallTwoFull: // Picked up the balls, drive to shoot
                    stopIntake();
                    follower.followPath(paths.ToBallTwoBack);
                    setPathState(Paths.PathState.ToBallTwoBack);
                    break;

                case ToBallTwoBack: // Drive to shooting position
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
                    finishedAutonomous = true;
                    break;
            }
        }
    }


    static class Paths
    {

        public Pose startPose = new Pose(123, 125, Math.toRadians(90));
        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallTwoBack, ToBallOneBack, ballBallsShoot, ToBallTwo, ToBallTwoFull, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2,
                bottomBalls, bottomBallsEat, FinalPose, finalPose;

        public Paths(Follower follower, AutoStrat strat)
        {
            switch (strat)
            {
                case BASIC:
                    buildPathsBasic(follower);
                    break;
                case REGULAR:
                default:
                    buildPathsReg(follower);
                    break;
            }
        }

        private void buildPathsBasic(Follower follower)
        {
            startPose = new Pose(80, 8.3, Math.toRadians(90));
            final Pose shootPose = new Pose(80, 8.3);
            final Pose leavePose = new Pose(80, 50);

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setTangentHeadingInterpolation()
                    .build();

            FinalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, leavePose))
                    .setConstantHeadingInterpolation(Math.toRadians(142))
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
                            new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder().addPath(
                            new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToBallOneBack = follower.pathBuilder().addPath(
                            new BezierLine(ballOneFullPose, ballOneLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToShoot_1 = follower.pathBuilder().addPath(
                            new BezierLine(ballOneLinePose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();

            ToBallTwo = follower.pathBuilder().addPath(
                            new BezierLine(shootPose, ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(
                            new BezierLine(ballTwoLinePose, ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToBallTwoBack = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoFullPose, ballTwoLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(ballTwoLinePose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            ToThree = follower.pathBuilder().addPath(
                            new BezierLine(shootPose, ballThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            ToThreeFull = follower.pathBuilder().addPath(
                            new BezierLine(ballThreeLinePose, ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            bottomBalls = follower.pathBuilder().addPath(
                            new BezierLine(ballThreeFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                    .build();

            ballBallsShoot = follower.pathBuilder().addPath(
                            new BezierLine(shootPose, gateLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                    .build();

            bottomBallsEat = follower.pathBuilder().addPath(
                            new BezierLine(gateLinePose, gatePushPose))
                    .setConstantHeadingInterpolation(Math.toRadians(136))
                    .build();

            finalPose = follower.pathBuilder().addPath(
                            new BezierLine(gateLinePose, randomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(136))
                    .build();
        }

        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToBallTwoBack, ToBallOneBack, ToBallTwo, ToBallTwoFull,
            ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3,
            bottomBallsEat, finalPose, FinalPose
        }

    }

}