package org.firstinspires.ftc.teamcode.util.dashboard;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import java.util.HashMap;

/**
 * Utility class for graphing Turret data on the Panels Dashboard Graph.
 * Graphs the current motor position and the error (bearing) to the target.
 *
 * @author Vinegar Vikings - 5393
 */
public class Graph
{
    private static TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private static final HashMap<String, Double> graphData = new HashMap<>();

    public static void put(String key, double value)
    {
        graphData.put(key, value);
    }

    /**
     * Pushes Turret data to Telemetry for graphing.
     * Call this inside your TeleOp loop or BaseStable.displayTelemetry().
     */
    public static void update()
    {
        graphData.forEach((key, value) -> panelsTelemetry.addData(key, value));
        panelsTelemetry.update();

        graphData.clear();
    }
}