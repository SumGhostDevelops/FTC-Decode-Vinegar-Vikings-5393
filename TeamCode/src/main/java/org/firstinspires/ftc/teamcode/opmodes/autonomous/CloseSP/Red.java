package org.firstinspires.ftc.teamcode.opmodes.autonomous.CloseSP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;

@Autonomous(name = "SpecialCloseRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends AutoBase
{
    private Paths paths;

    @Override
    protected StrategyOption[] getStrategyOptions()
    {
        return new StrategyOption[]{
                new StrategyOption(AutoStrat.BASIC, "DPAD UP", "3 Ball (BASIC)"),
                new StrategyOption(AutoStrat.REGULAR, "DPAD RIGHT", "9 Ball (REGULAR)")
        };
    }

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
                    setPathState(Blue.Paths.PathState.ToBallOne);
                    break;
                case ToBallOne:
                    stopIntake();
                    follower.followPath(paths.ToBallOneFull);
                    setPathState(Blue.Paths.PathState.ToBallOneFull);
                case ToBallOneFull:

                    follower.followPath(paths.finalPose);
                    setPathState(Blue.Paths.PathState.finalPose);
                case finalPose:
                    finishedAutonomous = true;
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
        // Not implemented for this autonomous
    }


    static class Paths
    {
        public Pose startPose = new Pose(21.000, 125.000, Math.toRadians(0));

        public PathChain ToShoot,
                ToBallOne, ToBallOneFull, ToBallOneBack, ToBallTwo, ToBallTwoFull, ToBallTwoBack, ToThree,
                ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, finalPose;

        public Paths(Follower follower, AutoStrat autoStrat)
        {
            switch (autoStrat)
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
            // --- Pose definitions ---
            startPose = new Pose(82.000, 9.000, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(88.000, 36.000);
            final Pose ballOneFullPose = new Pose(129.000, 36.000);

            final Pose leavePose = new Pose(94.000, 58.000);


// --- Paths ---

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToBallOneFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, startPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, leavePose))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        private void buildPathsReg(Follower follower)
        {
            // --- Pose definitions ---
            startPose = new Pose(80.000, 8.300, Math.toRadians(90));

            final Pose ballOneLinePose = new Pose(95, 36.000);
            final Pose ballOneFullPose = new Pose(125.000, 36.000);

            final Pose ballTwoLinePose = new Pose(95, 60.000);
            final Pose ballTwoFullPose = new Pose(125.000, 60.000);

            final Pose ballThreeLinePose = new Pose(95, 84.000);
            final Pose ballThreeFullPose = new Pose(125.000, 84.000);

            final Pose shootPose = new Pose(80.000, 8.300);
            final Pose finalPosePoint = new Pose(102.000, 41.000);


// --- Paths ---

            ToBallOne = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, ballOneLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            ToBallOneFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneLinePose, ballOneFullPose))
                    .setTangentHeadingInterpolation()

                    .build();

            ToShoot_1 = follower.pathBuilder()
                    .addPath(new BezierLine(ballOneFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();


            ToBallTwo = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballTwoLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            ToBallTwoFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoLinePose, ballTwoFullPose))
                    .setTangentHeadingInterpolation()

                    .build();

            ToShoot_2 = follower.pathBuilder()
                    .addPath(new BezierLine(ballTwoFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();


            ToThree = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, ballThreeLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(88), Math.toRadians(0))
                    .build();

            ToThreeFull = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeLinePose, ballThreeFullPose))
                    .setTangentHeadingInterpolation()

                    .build();

            ToShoot_3 = follower.pathBuilder()
                    .addPath(new BezierLine(ballThreeFullPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();


            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, finalPosePoint))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }


        public enum PathState
        {
            ToShoot, ToBallOne, ToBallOneFull, ToBallOneBack, ToBallTwo, ToBallTwoFull, ToBallTwoBack,
            ToThree, ToThreeFull, ToShoot_1, ToShoot_2, ToShoot_3, finalPose
        }
    }
}
