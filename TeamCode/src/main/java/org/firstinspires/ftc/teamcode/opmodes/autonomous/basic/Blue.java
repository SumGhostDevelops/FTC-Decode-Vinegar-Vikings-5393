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
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.definitions.PedroConstants;

@Autonomous(name = "BlueBasicVikingsAutonomous", group = "Blue", preselectTeleOp = "BlueVikingsTeleOp")
public class Blue extends Base
{
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
         }
    }


