package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.definitions.PedroConstants;

@Autonomous(name = "RedBasicVikingsAutonomous", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends Base
{
    private AutoPaths paths;
    private Timer timer, opModeTimer;
    private Follower follower;

    private AutoPaths.PathState currentPathState;

    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        team = Team.RED;
        RobotContext robotContext = new RobotContext(team,hardwareMap,telemetry,gamepad1,gamepad2);
        if (opModeIsActive() && !isStopRequested()) {
            opModeTimer.resetTimer();
            robotContext.subsystems.transfer.open();
            robotContext.subsystems.outtake.setTargetRPM(4000);
            robotContext.subsystems.outtake.on();

            for (int i = 0; i < 4; i++)
            {
                while (robotContext.subsystems.outtake.isReady())
                {
                    robotContext.subsystems.intake.in(0.6);
                }
                robotContext.subsystems.intake.stop();
                Thread.sleep(3000);
            }

            follower.followPath(paths.ToShoot);

            while (opModeIsActive() && !isStopRequested()) {

                follower.update();

                telemetry.addData("Current State", currentPathState);
                telemetry.addData("State Time (s)", timer.getElapsedTimeSeconds());
                telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
                telemetry.update();
            }
        }
    }

    public void initAuto() throws InterruptedException {
        follower = PedroConstants.createFollower(hardwareMap);
        paths = new AutoPaths(follower);

        // Try to set follower pose to the path start pose (works for different follower APIs)
        try {
            java.lang.reflect.Method m = follower.getClass().getMethod("setPose", com.pedropathing.geometry.Pose.class);
            m.invoke(follower, paths.startPose);
        } catch (NoSuchMethodException e) {
            try {
                java.lang.reflect.Method m2 = follower.getClass().getMethod("setPoseEstimate", com.pedropathing.geometry.Pose.class);
                m2.invoke(follower, paths.startPose);
            } catch (Exception ignored) {}
        } catch (Exception ignored) {}

        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        timer.resetTimer();
    }

    public void setPathState(AutoPaths.PathState pathState) {
        currentPathState = pathState;
        timer.resetTimer();
    }

    public static class AutoPaths {

        public PathChain ToShoot;
        public final Pose startPose;

        public enum PathState {
            ToShoot
        }

        public AutoPaths(Follower follower) {
            startPose = new Pose(63, 8.3);
            final Pose shootPose = new Pose(57, 44);

            ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    startPose,
                                    shootPose
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }
}
