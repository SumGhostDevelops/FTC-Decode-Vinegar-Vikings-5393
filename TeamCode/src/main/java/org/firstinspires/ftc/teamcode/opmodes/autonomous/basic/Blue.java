package org.firstinspires.ftc.teamcode.opmodes.autonomous.basic;
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
import org.firstinspires.ftc.teamcode.definitions.RobotContext;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.definitions.PedroConstants;

import java.nio.file.Paths;


@Autonomous(name = "BlueBasicVikingsAutonomous", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends Base {


    private Paths paths;
    private Timer timer, opModeTimer;
    private Follower follower;


    private Paths.PathState currentPathState;

    public void runOpMode() throws InterruptedException {
        team = Team.BLUE;
        RobotContext robotContext = new RobotContext(team,hardwareMap,telemetry,gamepad1,gamepad2);
        initAuto();
        waitForStart();
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
            final Pose startPose = new Pose(80, 8.3);
            final Pose shootPose = new Pose(92, 51);


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

