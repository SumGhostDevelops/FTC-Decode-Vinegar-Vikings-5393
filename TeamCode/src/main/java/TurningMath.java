import java.lang.Math;
public class TurningMath
{
    static double timeToTurnNinety = 0.204;
    static double Lx = 0.410;
    static double Ly = 0.336;
    static double r = 0.052;
    public static double Calculate (double degrees)
    {
        /*
        double radiansToTurn = (Math.sqrt(Math.pow(Lx,2)+Math.pow(Ly,2))*degrees)/r;
        double degreesToTurn = Math.toDegrees(radiansToTurn);
        double timeToActivate = timeToTurnNinety * (degreesToTurn/90);
        return timeToActivate;
        */
        return Math.abs(timeToTurnNinety * (degrees / 90.0));
    }

}
