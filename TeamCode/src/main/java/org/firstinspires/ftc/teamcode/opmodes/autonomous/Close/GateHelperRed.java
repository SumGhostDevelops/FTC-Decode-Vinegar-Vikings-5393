package org.firstinspires.ftc.teamcode.opmodes.autonomous.Close;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.definitions.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;


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
      //  setPathState(Paths.PathState.toShoot);

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

    static class Paths{

        public Paths(Follower follower, AutoStrat strat)
        {
            switch (strat)
            {
            }
        }
        public Pose startPose = new Pose(80, 8.3, Math.toRadians(90));

        public PathChain toShoot;


        public enum PathState
        {

        }
    }


}
