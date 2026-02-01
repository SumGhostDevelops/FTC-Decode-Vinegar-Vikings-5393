package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Utility class for persisting autonomous end-state (pose + turret position) to
 * file storage. TeleOp can load this state if it was saved recently (within a
 * configurable time window).
 */
public class AutoStateStorage
{
	private static final String FILENAME = "auto_state.txt";

	/**
	 * Data class holding the saved autonomous state.
	 */
	public static class AutoState
	{
		public final double x;
		public final double y;
		public final double headingRadians;
		public final double turretDegrees;
		public final long timestamp;

		public AutoState(double x, double y, double headingRadians, double turretDegrees, long timestamp)
		{
			this.x = x;
			this.y = y;
			this.headingRadians = headingRadians;
			this.turretDegrees = turretDegrees;
			this.timestamp = timestamp;
		}
	}

	/**
	 * Saves the current autonomous state to file storage.
	 *
	 * @param x              Robot x-coordinate (Pedro Pathing units)
	 * @param y              Robot y-coordinate (Pedro Pathing units)
	 * @param headingRadians Robot heading in radians
	 * @param turretDegrees  Turret position in degrees (from motor encoder)
	 */
	public static void saveState(double x, double y, double headingRadians, double turretDegrees)
	{
		try
		{
			File file = AppUtil.getInstance().getSettingsFile(FILENAME);
			FileWriter writer = new FileWriter(file);

			long timestamp = System.currentTimeMillis();
			// Format: timestamp,x,y,heading,turretDegrees
			writer.write(String.format("%d,%f,%f,%f,%f", timestamp, x, y, headingRadians, turretDegrees));
			writer.close();
		}
		catch (IOException e)
		{
			// Silently fail - state persistence is non-critical
			e.printStackTrace();
		}
	}

	/**
	 * Loads a recently saved autonomous state.
	 *
	 * @param maxAgeMillis Maximum age of the saved state in milliseconds. If the
	 *                     state is older than this, null is returned.
	 * @return The saved AutoState if recent and valid, otherwise null.
	 */
	public static AutoState loadRecentState(long maxAgeMillis)
	{
		try
		{
			File file = AppUtil.getInstance().getSettingsFile(FILENAME);
			if (!file.exists())
			{
				return null;
			}

			BufferedReader reader = new BufferedReader(new FileReader(file));
			String line = reader.readLine();
			reader.close();

			if (line == null || line.isEmpty())
			{
				return null;
			}

			String[] parts = line.split(",");
			if (parts.length != 5)
			{
				return null;
			}

			long timestamp = Long.parseLong(parts[0]);
			long age = System.currentTimeMillis() - timestamp;

			if (age > maxAgeMillis)
			{
				return null; // State is too old
			}

			double x = Double.parseDouble(parts[1]);
			double y = Double.parseDouble(parts[2]);
			double headingRadians = Double.parseDouble(parts[3]);
			double turretDegrees = Double.parseDouble(parts[4]);

			return new AutoState(x, y, headingRadians, turretDegrees, timestamp);
		}
		catch (IOException | NumberFormatException e)
		{
			// Silently fail - state persistence is non-critical
			e.printStackTrace();
			return null;
		}
	}

	/**
	 * Clears any saved state file.
	 */
	public static void clearState()
	{
		try
		{
			File file = AppUtil.getInstance().getSettingsFile(FILENAME);
			if (file.exists())
			{
				file.delete();
			}
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}
}
