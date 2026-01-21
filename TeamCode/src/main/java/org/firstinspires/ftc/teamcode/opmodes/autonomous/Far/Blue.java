package org.firstinspires.ftc.teamcode.opmodes.autonomous.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;

@Autonomous(name = "FarBlueAuto", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class
Blue extends Base {


    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;

    private Paths.PathState currentPathState;
    private AutoStrat autoStrat = AutoStrat.REGULAR;


    @Override
   public void runOpMode() throws InterruptedException {
        team = Team.BLUE;
        super.runOpMode();
        while (opModeInInit()) {
            // --- Move your selection logic inside this loop ---
            if (gamepad1.dpad_up) {
                autoStrat = AutoStrat.GATE;
            } else if (gamepad1.dpad_right) {
                autoStrat = AutoStrat.REGULAR;
            } else if (gamepad1.dpad_down) {
                autoStrat = AutoStrat.BASIC;
            }

            // --- Keep your telemetry inside the loop for live feedback ---
            telemetry.addLine("--- SELECT AUTO STRATEGY ---");
            telemetry.addData("Selected", autoStrat);
            telemetry.addLine("\nControls:");
            telemetry.addLine("DPAD UP: 12 Ball (GATE)");
            telemetry.addLine("DPAD RIGHT: 9 Ball (REGULAR)");
            telemetry.addLine("DPAD DOWN: 3 Ball (BASIC)");
            telemetry.update();
        }

        // Now the selection is locked in. Initialize paths and timers.
        initAuto();

        // The OpMode will wait here until you press START
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            opModeTimer.resetTimer();


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
    //    paths = new Paths(follower);
        timer = new Timer();

      //  follower.setStartingPose(paths.startPose);
        follower.update();

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

        public void Paths(){


        }



    }
}