import org.usfirst.frc.team6911.robot.navigation.Path;
import org.usfirst.frc.team6911.robot.navigation.Point;

public class MainClass
{
	public static void main(String[] args)
	{
		Point[] userIn = new Point[] {new Point(0,0,0), new Point(128.54,0), 
				new Point(128.54,-87.625), new Point(141.27, -100.355)};
		double weight_smooth = 0.80;
		double a = 1 - weight_smooth;
		double tolerance = 0.001;
		Path path = new Path(userIn);
		
		Path genPath = new Path(path.generatePath(path.numPointForArray(6)));
		genPath = genPath.smoother( a, weight_smooth, tolerance);
		
		//path.fullGeneration(1, a, weight_tolerance, tolerance);
		
		genPath.setTarVel();

		for(Point x: genPath)
			System.out.println(x);
	}
}