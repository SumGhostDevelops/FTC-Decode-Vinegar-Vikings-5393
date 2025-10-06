import java.lang.Math;
public class TurningMath
{
    double Lx = 1;
    double Ly = 1;
    double r = 1;
    public void Calculate (double degrees)
    {
        double radiansToTurn = (Math.sqrt(Math.pow(Lx,2)+Math.pow(Lx,2))*degrees)/r;
    }

}
