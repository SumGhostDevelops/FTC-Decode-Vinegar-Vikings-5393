package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.teamcode.util.measure.angle.Angle;
import org.firstinspires.ftc.teamcode.util.measure.angle.UnnormalizedAngle;

public class Turret extends SubsystemBase
{
    private final MotorEx turretMotor;
    private final double degreePerTick = 360.0 / (RobotConstants.Turret.PPR * RobotConstants.Turret.GEAR_RATIO);
    private Angle initialRelativeAngle;

    private int targetPosition = 0;

    /**
     * If true, the turret will continuously aim to the last {@link Turret#targetPosition} specified when {@link Turret#periodic()} is called.
     */
    public boolean lockToPosition = true;

    public Turret(MotorEx turretMotor, Angle initialRelativeAngle, boolean shouldLock)
    {
        this(turretMotor, initialRelativeAngle);
        this.lockToPosition = shouldLock;
    }
    public Turret(MotorEx turretMotor, Angle initialRelativeAngle)
    {
        this.turretMotor = turretMotor;
        this.initialRelativeAngle = initialRelativeAngle;
    }

    /**
     * Aim absolutely to the field, around the reported {@code initialRelativeAngle}
     * @param targetAngle The absolute target angle
     * @param robotAngle The robot's absolute angle
     * @see #setTargetRelative(Angle targetAngle)
     */
    public void setTargetAbsolute(Angle targetAngle, Angle robotAngle)
    {
        setTargetRelative(targetAngle.minus(robotAngle));
    }

    /**
     * Aim relatively to the robot, around the reported {@code initialRelativeAngle}
     * @param targetAngle The desired turret angle relative to the robot's orientation
     * @see #setTargetAbsolute(Angle targetAngle, Angle robotAngle)
     */
    public void setTargetRelative(Angle targetAngle)
    {
        int targetTicks = resolveSolution(turretMotor.getCurrentPosition(), targetAngle);

        // Get turn limits
        UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;

        // Final safety clamp (should not be needed if logic above is correct)
        int minTicks = angleToTicks(turnLimits[0]);
        int maxTicks = angleToTicks(turnLimits[1]);
        this.targetPosition = Math.max(minTicks, Math.min(maxTicks, targetTicks));
    }

    /**
     * Aims to the tick specified in {@link Turret#targetPosition}
     */
    public void aim()
    {
        // Use SDK's RUN_TO_POSITION for built-in deceleration and position holding
        turretMotor.motorEx.setTargetPosition(targetPosition);
        turretMotor.motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.motorEx.setPower(1.0);
    }

    /**
     * @param robotAngle
     * @return The absolute {@link Angle} of the turret based on the relative {@link Angle} and the robot's {@link Angle}
     * @see #getAbsoluteUnnormalizedAngle(UnnormalizedAngle)
     */
    public Angle getAbsoluteAngle(Angle robotAngle)
    {
        return getRelativeAngle().plus(robotAngle);
    }

    /**
     *
     * @param robotAngle
     * @return The absolute {@link UnnormalizedAngle} of the turret based on the relative {@link UnnormalizedAngle} and the robot's {@link UnnormalizedAngle}
     * @see #getAbsoluteAngle(Angle)
     */
    public UnnormalizedAngle getAbsoluteUnnormalizedAngle(UnnormalizedAngle robotAngle)
    {
        return getRelativeUnnormalizedAngle().plus(robotAngle);
    }

    /**
     * @return The relative {@link Angle} of the turret
     * @see #getRelativeUnnormalizedAngle()
     */
    public Angle getRelativeAngle()
    {
        return getRelativeUnnormalizedAngle().toNormalized();
    }

    /**
     * @return The {@link UnnormalizedAngle} of the relative turret heading in the raw {@code UnnormalizedAngleUnit.DEGREES}
     * @see #getRelativeAngle()
     */
    public UnnormalizedAngle getRelativeUnnormalizedAngle()
    {
        double motorPosition = turretMotor.getCurrentPosition(); // in ticks
        double turretAngle = degreePerTick * motorPosition; // in degrees

        return initialRelativeAngle.toUnit(UnnormalizedAngleUnit.DEGREES) // convert to an unnormalized angle
                .plus(new UnnormalizedAngle(turretAngle, UnnormalizedAngleUnit.DEGREES)); // add the turret angle to the initial angle
    }

    public void reset()
    {
        initialRelativeAngle = RobotConstants.Turret.FORWARD_ANGLE;
    }

    public Command turnToCommand(Angle angle)
    {
        return new InstantCommand(() -> this.setTargetRelative(angle), this);
    }

    /**
     * Resolves the best turret encoder position (in ticks) to reach the requested angle
     * while respecting the configured turn limits.
     *
     * @param currentTicks The current turret position in encoder ticks.
     * @param targetAngle  The desired turret angle to turn to.
     * @return The solution, in ticks
     */
    private int resolveSolution(int currentTicks, Angle targetAngle)
    {
        // Convert current position to unnormalized degrees
        double currentDegrees = degreePerTick * currentTicks;

        // Get target in degrees (normalized [-180, 180))
        double targetDegrees = targetAngle.getAngle(AngleUnit.DEGREES);

        // Get turn limits in degrees
        UnnormalizedAngle[] turnLimits = RobotConstants.Turret.TURN_LIMITS;
        double minDegrees = turnLimits[0].getAngle(UnnormalizedAngleUnit.DEGREES);
        double maxDegrees = turnLimits[1].getAngle(UnnormalizedAngleUnit.DEGREES);

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
        return (int) Math.round(bestTarget / degreePerTick);
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
        if (!turretMotor.motorEx.isBusy()) // check if the motor is currently going to a target
        {
            if (this.lockToPosition) // if not, and we should be "locked" to a position
            {
                aim(); // aim to that position
            }
            else // if the motor is done and we shouldn't continuously aim
            {
                turretMotor.motorEx.setPower(0); // set the turret power to 0
            }
        }
    }
}