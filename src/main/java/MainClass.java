import java.util.ArrayList;

import org.usfirst.frc.team6911.robot.navigation.Path;
import org.usfirst.frc.team6911.robot.navigation.Point;

public class MainClass
{

	public static void main(String[] args)
	{		
		Point[] userIn = new Point[] {new Point(1,3),new Point(6,3),new Point(6,5),new Point(11,5),new Point(11,10), new Point(9,10)};
		
		Path path = new Path(userIn);
		ArrayList<Point> genPath = new ArrayList<Point>();
		
		genPath = path.generatePath(path.numPointForArray(1));
		
		for(Point gen: genPath)
			System.out.println(gen);
	}

}