import org.usfirst.frc.team6911.robot.navigation.Path;
import org.usfirst.frc.team6911.robot.navigation.Point;

public class MainClass
{
	public static void main(String[] args)
	{
		double weight_smooth = 0.80;
		double a = 1 - weight_smooth;
		double tolerance = 0.001;
		
		Path path = (new Path(new Point[] {new Point(1,1,0), new Point(129.54,1), 
				new Point(129.54,-86.625), new Point(142.27, -99.355)}));
		
		Path genPath = new Path(path.generatePath(path.numPointForArray(6)));
		
		genPath = genPath.smoother( a, weight_smooth, tolerance);
		genPath.setTarVel();

		for(Point x: genPath)
			System.out.println(x);
	}
}