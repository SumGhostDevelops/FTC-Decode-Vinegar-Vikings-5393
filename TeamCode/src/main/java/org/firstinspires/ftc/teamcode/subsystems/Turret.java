package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

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

    public Turret(MotorEx turretMotor, Angle initialRelativeHeading)
    {
        this.turretMotor = turretMotor;
        this.initialRelativeHeading = initialRelativeHeading;
    }

    public void aimAbsolute(Angle targetAngle, Angle robotHeading)
    {
        aimRelative(targetAngle.minus(robotHeading));
    }

    public void aimRelative(Angle targetAngle)
    {
        Angle currentHeading = getRelativeHeading(AngleUnit.DEGREES);

        // Find how much we should move from our current heading
        Angle headingDelta = findBestTurretSolution(currentHeading, targetAngle);

        // Convert to ticks (the quantity of the motor)
        int ticksDelta = angleToTicks(headingDelta);

        int solution = turretMotor.getCurrentPosition() + ticksDelta;
        turretMotor.setTargetPosition(solution);
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
        Angle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
        angle = angle.toUnit(turnLimits[0].unit);

        return turnLimits[0].angle <= angle.angle && angle.angle <= turnLimits[1].angle;
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
            Angle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
            AngleUnit unit = turnLimits[0].unit;
            double clampedAngle = MathUtils.clamp(shortAngle.toUnitOnlyAngle(unit), turnLimits[0].angle, turnLimits[1].angle);
            return new Angle(clampedAngle - currentAngle.toUnitOnlyAngle(unit), unit);
        }
    }

    private int angleToTicks(Angle angle)
    {
        return RobotMath.Motor.angleToTicks(angle, RobotConstants.Turret.PPR, RobotConstants.Turret.GEAR_RATIO);
    }
}