package org.firstinspires.ftc.teamcode.util.dashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Utility class for graphing Turret data on the Panels Dashboard Graph.
 * Graphs the current motor position and the error (bearing) to the target.
 *
 * @author Vinegar Vikings - 5393
 */
public class TurretGraph
{
    private static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    /**
     * Pushes Turret data to Telemetry for graphing.
     * Call this inside your TeleOp loop or BaseStable.displayTelemetry().
     *
     * @param turret    The turret subsystem
     */
    public static void update(Turret turret)
    {
        if (turret == null) return;

        // We use Unnormalized angles for the position to prevent graph wrapping/spikes
        // when crossing 0/360 degrees, which makes tuning PID much easier.
        double currentPosition = turret.getRelativeUnnormalizedAngle().getDegrees();

        // "Bearing to target" is effectively the error (Target - Current)
        double bearingToTarget = turret.bearingToTarget().getDegrees();

        // Panels Graph reads "Key:Value"
        panelsTelemetry.addData("Turret:Pos", currentPosition);
        panelsTelemetry.addData("Turret:Bearing", bearingToTarget);

        panelsTelemetry.update();
    }
}