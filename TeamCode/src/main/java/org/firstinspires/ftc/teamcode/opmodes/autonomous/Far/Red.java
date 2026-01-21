package org.firstinspires.ftc.teamcode.opmodes.autonomous.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;


@Autonomous(name = "FarRedAuto", group = "Red", preselectTeleOp = "RedVikingsTeleOp")
public class Red extends Base {

    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;

    private Paths.PathState currentPathState;

    private AutoStrat autoStrat = AutoStrat.REGULAR;

    @Override
    public void runOpMode() throws InterruptedException {

        team = Team.RED;
        super.runOpMode();


        if (opModeIsActive() && !isStopRequested()) {
            opModeTimer.resetTimer();
            //  follower.followPath(paths.ToShoot);


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
        paths = new Paths(follower);
        timer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        //  setPathState(Paths.PathState.ToShoot);

    }

    public void setPathState(Paths.PathState pathState) {
        currentPathState = pathState;
        timer.resetTimer();
    }

    private void handlePathing() {

    }

    public static class Paths {


        public enum PathState {


        }

        public Paths(Follower follower) {


        }

    }
}