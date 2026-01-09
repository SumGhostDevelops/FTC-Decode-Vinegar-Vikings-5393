package org.firstinspires.ftc.teamcode.opmodes.autonomous.full;

import android.provider.SyncStateContract;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.definitions.PedroConstants;


@Autonomous(name = "RedFullVikingsAutonomous", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends Base
{

    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;

    private Paths.PathState currentPathState;


    @Override
    public void runOpMode() throws InterruptedException
    {

        team = Team.RED;
        super.runOpMode();


        if (opModeIsActive() && !isStopRequested()) {
            opModeTimer.resetTimer();
            follower.followPath(paths.ToShoot);


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


    public void initAuto(){
        follower = PedroConstants.createFollower(hardwareMap);
        paths = new Paths(follower);
        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        setPathState(Paths.PathState.ToShoot);

    }
    public void setPathState(Paths.PathState pathState) {
        currentPathState = pathState;
        timer.resetTimer();
    }
    private void handlePathing() {
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
                ToThreeFull, ToRandom, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4;



        public enum PathState {
            ToShoot, ToBallOne, ToBallOneFull, ToBallTwo, ToBallTwoFull, Gate1, Gate2, ToEatGate, ToThree,
            ToThreeFull, ToRandom, ToShoot_1, ToShoot_2, ToShoot_3, ToShoot_4


        }
        public Paths(Follower follower) {
            final Pose startPose = new Pose(124.000, 123.000);
            final Pose shootPose = new Pose(90.0, 97.2);
            final Pose ballOneLinePose = new Pose(90.0, 84.0);
            final Pose ballOneFullPose = new Pose(125.0, 84.0);
            final Pose ballTwoLinePose = new Pose(90.000, 58.000);
            final Pose ballTwoFullPose =   new Pose(125.000, 58.000);
            final Pose gateLinePose =  new Pose(121.000, 70.000);
            final Pose gatePushPose =  new Pose(128.000, 68.000);
            final Pose gateEatPose = new Pose(133.000, 60.000);
            final Pose ballThreePose =  new Pose(90.000, 35.000);
            final Pose ballThreeFullPose =  new Pose(125.000, 35.000);
            final Pose RandomPose =   new Pose(103.393, 95.740);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,

                                    shootPose
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ToBallOne = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballOneLinePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

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
                    ).setConstantHeadingInterpolation(Math.toRadians(37))

                    .build();

            ToBallTwo = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballTwoLinePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();

            ToBallTwoFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoLinePose,

                                    ballTwoFullPose
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToShoot_2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballTwoFullPose,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                    .build();

            Gate1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    gateLinePose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Gate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateLinePose,

                                    gatePushPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            ToEatGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gatePushPose,

                                    gateEatPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(55))

                    .build();

            ToShoot_3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gateEatPose,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(37))

                    .build();

            ToThree = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    ballThreePose
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();

            ToThreeFull = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreePose,

                                    ballThreeFullPose

                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToShoot_4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    ballThreeFullPose,

                                    shootPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(37))

                    .build();

            ToRandom = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    RandomPose
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(67))

                    .build();

        }











    }

}