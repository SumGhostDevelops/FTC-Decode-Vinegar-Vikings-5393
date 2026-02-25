package org.firstinspires.ftc.teamcode.definitions.constants;

public class ConstantsPresets
{
    public enum Preset
    {
        DEFAULT,
        TESTING,
        COMPETITION,
        COMPETITION_TESTING
    }

    /**
     * Applies the set {@link RobotConstants.General#PRESET_OPTION}
     */
    public static void applyPreset()
    {
        applyPreset(RobotConstants.General.PRESET_OPTION);
    }

    /**
     * Applies the provided {@link Preset}
     * @param preset
     */
    public static void applyPreset(Preset preset)
    {
        switch (preset)
        {
            case DEFAULT:
                break;
            case TESTING:
                RobotConstants.Outtake.ON_BY_DEFAULT = false;
                RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT = false;
                RobotConstants.General.REGRESSION_TESTING_MODE = true;
                RobotConstants.Turret.AUTO_AIM_TO_GOAL = false;
                RobotConstants.Intake.INTAKE_BY_DEFAULT = false;
                break;
            case COMPETITION:
                RobotConstants.Outtake.ON_BY_DEFAULT = true;
                RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT = true;
                RobotConstants.General.REGRESSION_TESTING_MODE = false;
                RobotConstants.Turret.AUTO_AIM_TO_GOAL = false;
                RobotConstants.Intake.INTAKE_BY_DEFAULT = false;
                RobotConstants.Telemetry.ENABLE_FIELD_DRAWING = false;
                RobotConstants.Telemetry.ENABLE_GRAPH_OUTPUT = false;
                break;
            case COMPETITION_TESTING:
                RobotConstants.Outtake.ON_BY_DEFAULT = true;
                RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT = true;
                RobotConstants.General.REGRESSION_TESTING_MODE = false;
                RobotConstants.Turret.AUTO_AIM_TO_GOAL = false;
                RobotConstants.Intake.INTAKE_BY_DEFAULT = false;
                RobotConstants.Telemetry.ENABLE_FIELD_DRAWING = true;
                RobotConstants.Telemetry.ENABLE_GRAPH_OUTPUT = true;
        }
    }
}
