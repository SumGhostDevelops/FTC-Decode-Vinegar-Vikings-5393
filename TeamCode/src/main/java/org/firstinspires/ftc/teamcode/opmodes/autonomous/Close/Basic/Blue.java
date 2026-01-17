// java
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

@Autonomous(name = "BlueMoveOnlyAuton", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends Base
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        this.team = Team.BLUE;
        super.runOpMode();
    }
}
