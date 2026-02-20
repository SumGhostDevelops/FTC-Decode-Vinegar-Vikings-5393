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
    protected void PathBasic()
    {
        if (!follower.isBusy())
        {
            switch (getPathState())
            {
                case ToShoot:
                    // Shoots first, then drives to final pose
                    Shoot();
                    startIntake();
                    follower.followPath(paths.ToBallOne, false);
                    setPathState(Paths.PathState.ToBallOne);
                    break;
                case ToBallOne:
                   stopIntake();
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Paths.PathState.ToBallOneFull);
                case ToBallOneFull:

                    follower.followPath(paths.finalPose);
                    setPathState(Paths.PathState.finalPose);
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
                case ToShoot: // initial preload shot
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
                    finishedAutonomous = true;
                    break;
            }
        }
    }


    static class Paths
    {

        public Pose startPose = new Pose(80, 8.3, Math.toRadians(90));
        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallTwoBack, ToBallOneBack, ballBallsShoot, ToBallTwo, ToBallTwoFull, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3,
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
            startPose =  new Pose(62.000, 9.000, Math.toRadians(90));
            final Pose toBallOne =  new Pose(56.000, 36.000);
            final Pose toBallOneFull =  new Pose(15.000, 36.000);
            final Pose leavePose =  new Pose(50.000, 58.000);

            ToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    toBallOne
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallOne = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    toBallOne,
                                    toBallOneFull
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ToBallOneFull = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    toBallOneFull,
                                    startPose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            finalPose = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    leavePose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

        }

        private void buildPathsReg(Follower follower)
        {
            // --- Pose definitions (updated numbers) ---
            startPose = new Pose(62.000, 9.000, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(50.000, 36.000);
            final Pose ballOneFullPose = new Pose(19.000, 36.000);

            final Pose ballTwoLinePose = new Pose(50.000, 60.000);
            final Pose ballTwoFullPose = new Pose(19.000, 60.000);

            final Pose ballThreeLinePose = new Pose(50.000, 84.000);
            final Pose ballThreeFullPose = new Pose(19.000, 84.000);

            final Pose shootPose = new Pose(62.000, 9.000);

            final Pose randomPose = new Pose(42.000, 41.000);


// --- Paths ---

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallOneFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();


            ToBallTwo = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToBallTwoFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoLinePose, ballTwoFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_2 = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();


            ToThree = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(92), Math.toRadians(180))
                    .build();

            ToThreeFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeLinePose, ballThreeFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_3 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();


            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, randomPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
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