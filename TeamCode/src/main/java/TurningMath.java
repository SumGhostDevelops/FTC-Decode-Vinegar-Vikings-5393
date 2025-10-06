import java.lang.Math;
public class TurningMath
{
    double timeToTurnNinety = 0.204;
    double Lx = 0.410;
    double Ly = 0.336;
    double r = 0.052;
    public double Calculate (double degrees)
    {
        double radiansToTurn = (Math.sqrt(Math.pow(Lx,2)+Math.pow(Lx,2))*degrees)/r;
        double degreesToTurn = Math.toDegrees(radiansToTurn);
        double timeToActivate = timeToTurnNinety * (degreesToTurn/90);
        return timeToActivate;
    }

}
