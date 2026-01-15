package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close.Basic;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;

@Autonomous(name = "BasicCloseRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends Base {

    @Configurable
    public static class Constants
    {
        public static double targetRPM = 4200;
        public static long lengthOfDrivingMs = 1000;
        public static long feedDuration = 300;
        public static long timeBetweenShots = 300;
    }

    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;

    private RobotContext robotContext; // initialize in initAuto
    private Paths.PathState currentPathState;

    @Override
    public void runOpMode() throws InterruptedException {
        team = Team.RED;

        initAuto();
        waitForStart();
        if (opModeIsActive() && !isStopRequested()) {
            opModeTimer.resetTimer();

            robotContext.subsystems.transfer.open();
            robotContext.subsystems.outtake.setTargetRPM(Constants.targetRPM);
            robotContext.subsystems.outtake.on();

            // Feed 4 rings: wait until shooter is ready, then pulse the intake to feed
            for (int i = 0; i < 4 && opModeIsActive() && !isStopRequested(); i++) {
                // wait until shooter reports ready
                while (opModeIsActive() && !isStopRequested() && !robotContext.subsystems.outtake.isReady()) {
                    Thread.sleep(10);
                }

                // pulse intake to feed one ring
                robotContext.subsystems.intake.in(0.6);
                Thread.sleep(Constants.feedDuration); // feed duration, tune as needed
                robotContext.subsystems.intake.stop();

                // short delay between shots
                Thread.sleep(Constants.timeBetweenShots);
            }

            /*
            // start path after shooting
            follower.followPath(paths.ToShoot);

            while (opModeIsActive() && !isStopRequested()) {
                follower.update();

                telemetry.addData("Current State", currentPathState);
                telemetry.addData("State Time (s)", timer.getElapsedTimeSeconds());
                telemetry.addData("OpMode Time (s)", opModeTimer.getElapsedTimeSeconds());
                telemetry.update();
            }

             */

            robotContext.subsystems.drive.setDrivePowers(1, 1, 1, 1);
            Thread.sleep(Constants.lengthOfDrivingMs);
            robotContext.subsystems.drive.stop();
        }
    }

    public void initAuto() throws InterruptedException {
        // follower and robotContext require hardwareMap and team to be set, so create them here
        follower = PedroConstants.createFollower(hardwareMap);
        robotContext = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);

        paths = new Paths(follower);
        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
    }

    public void setPathState(Paths.PathState pathState) {
        currentPathState = pathState;
        timer.resetTimer();
    }

    public static class Paths {
        public PathChain ToShoot;

        public enum PathState {
            ToShoot
        }

        public Paths(Follower follower) {
            // Tune these poses if the robot drives too far or in the wrong direction.
            final Pose startPose = new Pose(80, 8.3);
            final Pose shootPose = new Pose(80, 27); // adjust values if distance is incorrect

            ToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setTangentHeadingInterpolation()
                    .build();

        }
    }
}
