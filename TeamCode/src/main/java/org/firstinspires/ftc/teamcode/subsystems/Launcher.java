package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Launcher
{
    private final RobotHardware robot;
    private final ElapsedTime jamTimer = new ElapsedTime();
    private double targetRPM = RobotConstants.LAUNCHER_TARGET_RPM;
    private boolean isJamClearing = false;
    private double lastRpm = 0;

    public Launcher(RobotHardware robot)
    {
        this.robot = robot;
    }

    public void launch()
    {
        if (isJamClearing)
        {
            if (jamTimer.seconds() > RobotConstants.JAM_CLEAR_DURATION)
            {
                isJamClearing = false;
                robot.loaderServo.setPower(0);
            }
            else
            {
                robot.launcherMotor.setPower(RobotConstants.JAM_CLEAR_POWER);
                robot.loaderServo.setPower(-1.0); // Reverse loader
                return;
            }
        }

        // Simple P control or Feedforward would go here for velocity
        // For now, using basic setPower based on RPM assumption or built-in RUN_USING_ENCODER
        double currentVel = robot.launcherMotor.getVelocity(AngleUnit.DEGREES) / 6.0; // approx RPM conversion if ticks/rev known

        robot.launcherMotor.setVelocity(targetRPM); // Requires motor to be in RUN_USING_ENCODER

        // Shot detection logic
        if (lastRpm > 0 && currentVel < lastRpm * (1.0 - RobotConstants.SHOT_DROP_THRESHOLD))
        {
            // Ball launched
        }
        lastRpm = currentVel;
    }

    public void idle()
    {
        robot.launcherMotor.setPower(0);
        isJamClearing = false;
    }

    public void clearJam()
    {
        isJamClearing = true;
        jamTimer.reset();
    }

    public void setTargetAim(AprilTagDetection target)
    {
        if (target == null) return;
        // Distance calculation: hypotenuse of x and y
        double distance = Math.hypot(target.ftcPose.x, target.ftcPose.y);

        // Simple linear regression for RPM based on distance (example)
        targetRPM = 1500 + (distance * 10);
    }

    public double getRPM()
    {
        return robot.launcherMotor.getVelocity(); // Adjust unit as needed
    }
}