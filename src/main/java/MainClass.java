import org.usfirst.frc.team6911.robot.navigation.Path;
import org.usfirst.frc.team6911.robot.navigation.Point;

public class MainClass
{
	public static void main(String[] args)
	{
		Point[] userIn = new Point[] {new Point(1,1,0), new Point(5,1), new Point(9,12), new Point(15,6), new Point(19,12)};
		double weight_smooth = 0.85;
		double a = 1 - weight_smooth;
		double tolerance = 0.001;
		Path path = new Path(userIn);
		
		Path genPath = new Path(path.generatePath(path.numPointForArray(.25)));
		genPath = genPath.smoother( a, weight_smooth, tolerance);
		
		//path.fullGeneration(1, a, weight_tolerance, tolerance);
		
		genPath.setTarVel();

		for(Point x: genPath)
			System.out.println(x);
	}
}