package org.firstinspires.ftc.teamcode.definitions.constants;

public class ConstantsPresets
{
    public enum Preset
    {
        DEFAULT,
        TESTING,
        COMPETITION,
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
                RobotConstants.Intake.INTAKE_BY_DEFAULT = false;
                RobotConstants.General.REGRESSION_TESTING_MODE = true;
                break;
            case COMPETITION:
                RobotConstants.Outtake.ON_BY_DEFAULT = true;
                RobotConstants.Outtake.AUTO_DISTANCE_ADJUSMENT = true;
                RobotConstants.Turret.autoAimToGoal = true;
                RobotConstants.Intake.INTAKE_BY_DEFAULT = true;
                break;
        }
    }
}
