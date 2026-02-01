package org.firstinspires.ftc.teamcode.opmodes.autonomous.Far;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.definitions.hardware.RobotContext;
import org.firstinspires.ftc.teamcode.util.AutoStateStorage;

public abstract class Base extends LinearOpMode
{
    protected Team team;
    protected RobotContext robot;

    protected enum AutoStrat
    {
        BASIC, REGULAR, GATE
    }

    /**
     * Initializes RobotContext for hardware access. Call this in runOpMode() before
     * waitForStart().
     */
    protected void initRobotContext()
    {
        robot = new RobotContext(team, hardwareMap, telemetry, gamepad1, gamepad2);
    }

    /**
     * Saves the current autonomous state (pose + turret) for TeleOp to load. Call
     * this at the end of autonomous, after the main loop exits.
     *
     * @param follower The Pedro Pathing follower to get pose from
     */
    protected void saveAutoState(Follower follower)
    {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double heading = follower.getPose().getHeading();
        double turretDegrees = robot.subsystems.turret.getDistance();

        AutoStateStorage.saveState(x, y, heading, turretDegrees);
        telemetry.log().add("Saved auto state: (%.1f, %.1f) @ %.1f°", x, y, Math.toDegrees(heading));
    }

    public void Shoot()
    {
        // add logic to shoot
    }

    public void Intake()
    {
        // add logic to intake
    }

    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}