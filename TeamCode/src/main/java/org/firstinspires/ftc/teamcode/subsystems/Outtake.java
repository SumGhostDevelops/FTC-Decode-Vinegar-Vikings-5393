package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

public class Outtake extends Actuator
{
    private final RobotHardware robot;
    private int targetRPM = RobotConstants.OUTTAKE_INITIAL_TARGET_RPM;
    private double lastRPM = 0;
    private final ElapsedTime stabilityTimer = new ElapsedTime();

    public Outtake(RobotHardware robot)
    {
        super(robot.outtakeMotor);
        this.robot = robot;
    }

    public double getRPM()
    {
        return tpsToRpm(robot.outtakeMotor.getVelocity()); // Adjust unit as needed
    }

    public int getTargetRPM()
    {
        return targetRPM;
    }

    /**
     * Adds the provided RPM to the current target RPM
     * @param rpmChange
     */
    public void varyTargetRPM(int rpmChange)
    {
        if (targetRPM + rpmChange > 6000)
        {
            robot.telemetry.log().add("RPM was attempted to be set over the 6000 RPM limit (" + rpmChange + "). Capped to 6000.");
            rpmChange = 6000;
        }

        targetRPM += rpmChange;
    }

    /**
     * Changes the internal target RPM without turning the flywheel on
     * @param rpm
     */
    public void modifyTargetRPM(int rpm)
    {
        if (rpm > 6000)
        {
            robot.telemetry.log().add("RPM was attempted to be set over the 6000 RPM limit (" + rpm + "). Capped to 6000.");
            rpm = 6000;
        }

        targetRPM = rpm;
    }

    public void modifyTargetRPMBasedOnDistance(double distance)
    {
        setRPM(Math.toIntExact(Math.round(246.08237 * Math.pow(distance, 2) - 1653.96882 * distance + 6579.05937)));
        robot.telemetry.log().add("Auto-adjusted the RPM to " + targetRPM);
    }

    /**
     * Enables the flywheel with its currently set target RPM
     */
    public void setRPM()
    {
        setRPM(targetRPM);
    }

    public void setRPM(int rpm)
    {
        if (isResetting())
        {
            robot.telemetry.log().add("Ignoring a set RPM command because the motor is currently in a state of resetting.");
            return;
        }

        if (rpm > 6000)
        {
            robot.telemetry.log().add("RPM was attempted to be set over the 6000 RPM limit (" + rpm + "). Capped to 6000.");
            rpm = 6000;
        }

        status = Status.FORWARD_ENABLED;

        targetRPM = rpm;
        robot.outtakeMotor.setVelocity(rpmToTps(targetRPM));
    }

    /**
     * Sets the target RPM based on a distance regression and turns the flywheel on
     * @param distance
     */
    public void setRPMBasedOnDistance(double distance)
    {
        setRPM(Math.toIntExact(Math.round(246.08237 * Math.pow(distance, 2) - 1653.96882 * distance + 6579.05937)));
    }

    public boolean isReadyToLaunch()
    {
        double msSinceLastCheck = stabilityTimer.milliseconds();
        stabilityTimer.reset();

        if (msSinceLastCheck < 50)
        {
            return false;
        }

        double newRPM = getRPM();
        double delta_rpm = newRPM - lastRPM;
        lastRPM = newRPM;

        robot.telemetry.addData("Delta RPM", delta_rpm);

        boolean rpm_is_stable = (Math.abs(delta_rpm) < 10);
        boolean rpm_within_tolerance = Math.abs(getRPM() - targetRPM) < RobotConstants.OUTTAKE_RPM_TOLERANCE;

        return rpm_is_stable && rpm_within_tolerance && !isResetting();
    }

    // tps = Ticks per Second; rpm = Revolutions per Minute
    private double tpsToRpm(double ticks)
    {
        return (ticks * 60.0) / RobotConstants.OUTTAKE_PPR;
    }

    private double rpmToTps(double rpm)
    {
        return (rpm * RobotConstants.OUTTAKE_PPR) / 60.0;
    }
}