import java.lang.Math;
public class TurningMath
{
    double timeToTurnNinety = 0.204;
    double Lx = 1;
    double Ly = 1;
    double r = 1;
    public double Calculate (double degrees)
    {
        double radiansToTurn = (Math.sqrt(Math.pow(Lx,2)+Math.pow(Lx,2))*degrees)/r;
        double degreesToTurn = Math.toDegrees(radiansToTurn);
        double timeToActivate = timeToTurnNinety * (degreesToTurn/90);
        return timeToActivate;
    }

}
