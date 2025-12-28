package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.teamcode.util.measure.Angle;
import org.firstinspires.ftc.teamcode.util.measure.UnnormalizedAngle;

public class Turret extends SubsystemBase
{
    private final MotorEx turretMotor;
    private Angle initialRelativeHeading;
    private final double degreePerTick = 360.0 / (RobotConstants.Turret.PPR * RobotConstants.Turret.GEAR_RATIO);
    private int targetPosition = 0;
    private Telemetry telemetry;

    public Turret(MotorEx turretMotor, Angle initialRelativeHeading, Telemetry telemetry)
    {
        this.turretMotor = turretMotor;
        this.initialRelativeHeading = initialRelativeHeading;
        this.telemetry = telemetry;
    }

    public void aimAbsolute(Angle targetAngle, Angle robotHeading)
    {
        aimRelative(targetAngle.minus(robotHeading));
    }

    public void aimRelative(Angle targetAngle)
    {
        // Get current position in ticks and convert to unnormalized degrees
        int currentTicks = turretMotor.getCurrentPosition();
        double currentDegrees = degreePerTick * currentTicks;

        // Get target in degrees (normalized [-180, 180))
        double targetDegrees = targetAngle.toUnitOnlyAngle(AngleUnit.DEGREES);

        // Get turn limits in degrees
        UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
        double minDegrees = turnLimits[0].toUnitOnlyAngle(UnnormalizedAngleUnit.DEGREES);
        double maxDegrees = turnLimits[1].toUnitOnlyAngle(UnnormalizedAngleUnit.DEGREES);

        // Clamp current degrees to within limits for path calculation
        // This prevents getting stuck when slightly past a limit due to overshoot
        double clampedCurrentDegrees = Math.max(minDegrees, Math.min(maxDegrees, currentDegrees));

        // Find the best target position considering turn limits
        // Check equivalent angles (target + n*360) and pick the closest valid one
        // where the ENTIRE PATH from current to target stays within limits
        double bestTarget = clampedCurrentDegrees; // Default to staying in place if no valid target
        double bestDistance = Double.MAX_VALUE;

        for (int n = -2; n <= 2; n++) {
            double candidate = targetDegrees + (n * 360);

            // Check if this candidate is within turn limits
            if (candidate >= minDegrees && candidate <= maxDegrees) {
                // Also check that the path from current to candidate stays within limits
                double pathMin = Math.min(clampedCurrentDegrees, candidate);
                double pathMax = Math.max(clampedCurrentDegrees, candidate);

                // Path is valid if it doesn't go outside turn limits
                if (pathMin >= minDegrees && pathMax <= maxDegrees) {
                    double distance = Math.abs(candidate - clampedCurrentDegrees);
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestTarget = candidate;
                    }
                }
            }
        }

        // Convert best target to ticks
        int targetTicks = (int) Math.round(bestTarget / degreePerTick);

        // Final safety clamp (should not be needed if logic above is correct)
        int minTicks = angleToTicks(turnLimits[0]);
        int maxTicks = angleToTicks(turnLimits[1]);
        targetTicks = Math.max(minTicks, Math.min(maxTicks, targetTicks));

        targetPosition = targetTicks;

        // Debug telemetry
        telemetry.log().add("targetDeg=" + targetDegrees + " bestTarget=" + bestTarget + " currentDeg=" + currentDegrees);
        telemetry.log().add("Target: " + targetPosition + " current: " + currentTicks + " limits:[" + minTicks + "," + maxTicks + "]");

        // Use SDK's RUN_TO_POSITION for built-in deceleration and position holding
        turretMotor.motorEx.setTargetPosition(targetTicks);
        turretMotor.motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.motorEx.setPower(1.0);
    }

    public Angle getAbsoluteHeading(Angle robotHeading, AngleUnit finalUnit)
    {
        return robotHeading.plus(getRelativeHeading(finalUnit), finalUnit); // add the absolute heading of the robot with the relative heading of the turret
    }

    /**
     *
     * @param finalUnit
     * @return The NORMALIZED relative heading [-180 to 180)
     */
    public Angle getRelativeHeading(AngleUnit finalUnit)
    {
        return getRelativeHeading(finalUnit.getUnnormalized()).toUnit(finalUnit); // convert the unnormalized angle to the final unit
    }

    public void setPower(double power)
    {
        turretMotor.setRunMode(Motor.RunMode.RawPower);
        turretMotor.set(power);
    }

    /**
     *
     * @param finalUnit
     * @return The UNNORMALIZED relative heading
     */
    public UnnormalizedAngle getRelativeHeading(UnnormalizedAngleUnit finalUnit)
    {
        double motorPosition = turretMotor.getCurrentPosition(); // in ticks
        double angle = degreePerTick * motorPosition; // in degrees

        return initialRelativeHeading.toUnit(UnnormalizedAngleUnit.DEGREES).plus(new UnnormalizedAngle(angle, UnnormalizedAngleUnit.DEGREES)).toUnit(finalUnit); // add unnormalized angle to the initial heading
    }

    public void reset()
    {
        initialRelativeHeading = RobotConstants.Turret.FORWARD_ANGLE;
    }

    public Command turnToCommand(Angle angle)
    {
        return new InstantCommand(() -> this.aimRelative(angle), this);
    }

    private boolean validSolution(Angle angle)
    {
        UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
        UnnormalizedAngle unnormAngle = angle.toUnit(turnLimits[0].unit);

        return turnLimits[0].angle <= unnormAngle.angle && unnormAngle.angle <= turnLimits[1].angle;
    }

    /**
     *
     * @param targetAngle
     * @return The delta of the solution from the current
     */
    private Angle findBestTurretSolution(Angle currentAngle, Angle targetAngle)
    {
        // Calculate the two path options: shortest and longest
        Angle shortDelta = targetAngle.minus(currentAngle);
        Angle longDelta;

        if (shortDelta.angle > 0)
        {
            longDelta = shortDelta.minus(new Angle(360, AngleUnit.DEGREES));
        }
        else
        {
            longDelta = shortDelta.plus(new Angle(360, AngleUnit.DEGREES));
        }

        // Determine which path option is physically valid, prioritizing the shorter path
        Angle shortAngle = currentAngle.plus(shortDelta);
        Angle longAngle = currentAngle.plus(longDelta);

        boolean shortIsValid = validSolution(shortAngle);
        boolean longIsValid = validSolution(longAngle);

        if (shortIsValid && longIsValid) // If both are valid return the shortest one
        {
            if (Math.abs(shortDelta.angle) <= Math.abs(longDelta.angle))
            {
                return shortDelta;
            }
            else
            {
                return longDelta;
            }
        }
        // Return a valid solution, prioritizing the shortest path
        else if (shortIsValid)
        {
            return shortDelta;
        }
        else if (longIsValid)
        {
            return longDelta;
        }
        // Return a max angle if for some reason no solution is valid
        else
        {
            UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
            UnnormalizedAngleUnit unit = turnLimits[0].unit;
            double clampedAngle = MathUtils.clamp(shortAngle.toUnit(unit).angle, turnLimits[0].angle, turnLimits[1].angle);
            return new Angle(clampedAngle - currentAngle.toUnit(unit).angle, unit.getNormalized());
        }
    }

    private int angleToTicks(Angle angle)
    {
        return RobotMath.Motor.angleToTicks(angle, RobotConstants.Turret.PPR, RobotConstants.Turret.GEAR_RATIO);
    }

    private int angleToTicks(UnnormalizedAngle angle)
    {
        return RobotMath.Motor.angleToTicks(angle, RobotConstants.Turret.PPR, RobotConstants.Turret.GEAR_RATIO);
    }

    public int getTargetPosition()
    {
        return targetPosition;
    }

    @Override
    public void periodic()
    {
        // RUN_TO_POSITION mode: check if motor has reached target
        if (!turretMotor.motorEx.isBusy())
        {
            turretMotor.motorEx.setPower(0);
        }
    }
}