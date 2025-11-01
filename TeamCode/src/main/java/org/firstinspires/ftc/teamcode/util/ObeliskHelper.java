package org.firstinspires.ftc.teamcode.util;

public class ObeliskHelper
{
    public static String idToString(int obeliskId)
    {
        switch (obeliskId)
        {
            case 21:
                return "GPP";
            case 22:
                return "PGP";
            case 23:
                return "GGP";
            default:
                throw new IllegalArgumentException(obeliskId + " is not a valid obelisk ID.");
        }
    }

    public static int stringToId(String obeliskString)
    {
        switch (obeliskString)
        {
            case "GPP":
                return 21;
            case "PGP":
                return 22;
            case "GGP":
                return 23;
            default:
                throw new IllegalArgumentException(obeliskString + " is not a valid obelisk combination.");
        }
    }

    public static boolean isObelisk(int obeliskId)
    {
        if (obeliskId >= 21 && obeliskId <= 23)
        {
            return true;
        }

        return false;
    }
}