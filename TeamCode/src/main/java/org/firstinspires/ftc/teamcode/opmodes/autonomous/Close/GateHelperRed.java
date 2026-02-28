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

import java.util.Objects;


@Autonomous(name = "GateHelperRedAuto", group = " Red", preselectTeleOp = "RedVikingsTeleOp")
public class GateHelperRed extends AutoBase {
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
        setPathState(Paths.PathState.toShoot);

        // Wait for start
        waitForStart();

        // Run the main autonomous loop
        runAutoLoop();
    }

    @Override
    protected StrategyOption[] getStrategyOptions()
    {
        return new StrategyOption[]{
                new StrategyOption(AutoStrat.REGULAR, "DPAD RIGHT", "Helps with Gate Method Users (X Balls)"),

        };
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
    protected void PathRegular()
    {
        if (!follower.isBusy())
        {
            switch (getPathState())
            {
                case toShoot:
                    Shoot();
                    follower.followPath(paths.toShoot);
                    setPathState(Paths.PathState.Extra);
                    break;
                case Extra:
                    startIntake();
                    follower.followPath(paths.ToHit);
                    setPathState(Paths.PathState.ToHit);

                case ToHit:
                    follower.followPath(paths.Move);
                    setPathState(Paths.PathState.Move);
                    break;

                case Move:
                    stopIntake();
                    follower.followPath(paths.ToShoot);
                    setPathState(Paths.PathState.ToShoot);
                    break;

                case ToShoot:
                    Shoot();
                    follower.followPath(paths.Swing_1);
                    setPathState(Paths.PathState.Swing_1);
                    break;

                case Swing_1:
                    startIntake();
                    follower.followPath(paths.Hit_1);
                    setPathState(Paths.PathState.Hit_1);
                    break;

                case Hit_1:
                    follower.followPath(paths.Swing_2);
                    setPathState(Paths.PathState.Swing_2);
                    break;

                case Swing_2:
                    follower.followPath(paths.Hit_2);
                    setPathState(Paths.PathState.Hit_2);
                    break;

                case Hit_2:
                    stopIntake();
                    follower.followPath(paths.ToShoot_2);
                    setPathState(Paths.PathState.ToShoot_2);
                    break;

                case ToShoot_2:
                    Shoot();
                    follower.followPath(paths.ToSwingTwo_1);
                    setPathState(Paths.PathState.ToSwingTwo_1);
                    break;

                case ToSwingTwo_1:
                    startIntake();
                    follower.followPath(paths.HitTwo_1);
                    setPathState(Paths.PathState.HitTwo_1);
                    break;

                case HitTwo_1:
                    follower.followPath(paths.ToSwingTwo_2);
                    setPathState(Paths.PathState.ToSwingTwo_2);
                    break;

                case ToSwingTwo_2:
                    follower.followPath(paths.HitTwo_2);
                    setPathState(Paths.PathState.HitTwo_2);
                    break;

                case HitTwo_2:
                    stopIntake();
                    follower.followPath(paths.ToShoot_3);
                    setPathState(Paths.PathState.ToShoot_3);
                    break;

                case ToShoot_3:
                    Shoot();
                    follower.followPath(paths.ToSwingThree_1);
                    setPathState(Paths.PathState.ToSwingThree_1);
                    break;

                case ToSwingThree_1:
                    startIntake();
                    follower.followPath(paths.HitThree_1);
                    setPathState(Paths.PathState.HitThree_1);
                    break;

                case HitThree_1:
                    follower.followPath(paths.ToSwingThree_2);
                    setPathState(Paths.PathState.ToSwingThree_2);
                    break;

                case ToSwingThree_2:
                    follower.followPath(paths.HitThree_2);
                    setPathState(Paths.PathState.HitThree_2);
                    break;

                case HitThree_2:
                    stopIntake();
                    follower.followPath(paths.ToShoot_4);
                    setPathState(Paths.PathState.ToShoot_4);
                    break;

                case ToShoot_4:
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

    static class Paths{

        public Pose startPose = new Pose(80, 8.3, Math.toRadians(90));

        public PathChain toShoot, ToHit, Move,ToShoot,
                Swing_1, Hit_1, Swing_2, Hit_2, ToShoot_2,
                ToSwingTwo_1, HitTwo_1, ToSwingTwo_2, HitTwo_2,
                ToShoot_3, ToSwingThree_1, HitThree_1, ToSwingThree_2,
                HitThree_2, ToShoot_4, finalPose;

        public Paths(Follower follower, AutoStrat strat)
        {
            if (Objects.requireNonNull(strat) == AutoStrat.REGULAR) {

                buildPathsReg(follower);

            }
        }

        private void buildPathsReg(Follower follower) {
// --- Pose Definitions ---
            startPose = new Pose(80, 8,  Math.toRadians(90));

            final Pose shootPose     = new Pose(80, 8,  Math.toRadians(90));
            final Pose approachPose  = new Pose(133.000, 8.000);
            final Pose hitPose       = new Pose(124.000, 12.000);
            final Pose curveControl  = new Pose(130.000, 25.000);
            final Pose exitPose      = new Pose(134.000, 12.000);
            final Pose swingLinePose = new Pose(119.000, 24.000);
            final Pose swingFullPose = new Pose(134.000, 24.000);
            final Pose finalPosePose = new Pose(124.000, 12.000);
// --- Paths ---
            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, approachPose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            ToHit = follower.pathBuilder()
                    .addPath(new BezierLine(approachPose, hitPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(240))
                    .setReversed()
                    .build();

            Move = follower.pathBuilder()
                    .addPath(new BezierCurve(hitPose, curveControl, exitPose))
                    .setConstantHeadingInterpolation(Math.toRadians(240))
                    .build();

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(exitPose, shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(90))
                    .setReversed()
                    .build();

            Swing_1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, swingLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Hit_1 = follower.pathBuilder()
                    .addPath(new BezierLine(swingLinePose, swingFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            Swing_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingFullPose, swingLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Hit_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingLinePose, swingFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            ToSwingTwo_1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, swingLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            HitTwo_1 = follower.pathBuilder()
                    .addPath(new BezierLine(swingLinePose, swingFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToSwingTwo_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingFullPose, swingLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            HitTwo_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingLinePose, swingFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_3 = follower.pathBuilder()
                    .addPath(new BezierLine(swingFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            ToSwingThree_1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, swingLinePose))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            HitThree_1 = follower.pathBuilder()
                    .addPath(new BezierLine(swingLinePose, swingFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToSwingThree_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingFullPose, swingLinePose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            HitThree_2 = follower.pathBuilder()
                    .addPath(new BezierLine(swingLinePose, swingFullPose))
                    .setTangentHeadingInterpolation()
                    .build();

            ToShoot_4 = follower.pathBuilder()
                    .addPath(new BezierLine(swingFullPose, shootPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            finalPose = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, finalPosePose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

        }

        public enum PathState {
            toShoot,
            ToHit,
            Extra,
            Move,
            ToShoot,
            Swing_1,
            Hit_1,
            Swing_2,
            Hit_2,
            ToShoot_2,
            ToSwingTwo_1,
            HitTwo_1,
            ToSwingTwo_2,
            HitTwo_2,
            ToShoot_3,
            ToSwingThree_1,
            HitThree_1,
            ToSwingThree_2,
            HitThree_2,
            ToShoot_4,
            finalPose,
            END
        }

    }


}
