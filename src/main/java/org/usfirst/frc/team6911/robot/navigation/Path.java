package org.usfirst.frc.team6911.robot.navigation;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.drive.Vector2d;

@SuppressWarnings("serial")
public class Path extends ArrayList<Point>{

	/**
	The constructors for the Path class.
	**/
	
	public Path() {
		super();
	}

	public Path(Iterable<Point> path) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
	}
	
	public Path(Point[] path) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
	}
	
	/**
	This code will help with going from path to matrix, and back, as the code for 
	smoothing the path needs a matrix instead of the current ArrayList of Points
	**/

	/**
	What is happening in this code is instantiation, then a for loop where it loops 
	through each element of the ArrayList, and puts the X value for each point as 
	the first column, and the Y value for each point as the second column
	**/

	/**
	What is happening in this code is instantiation, then a for loop where it loops 
	through each row, and where the first column is the X value, and the second 
	column is the Y value, adds a new Point with those values of X and Y
	**/

	public double[][] pathToMatrix(){
		double[][] genPath = new double[size()][2];
		for(int i = 0; i < size(); i++){
			genPath[i][0] = get(i).getX();
			genPath[i][1] = get(i).getY();
		}
		return genPath;
	}
	
	public static Path matrixToPath(double[][] path){
		ArrayList<Point> temp = new ArrayList<Point>();
		for(int i = 0; i < path.length; i++)
			temp.add(new Point(path[i][0],path[i][1]));
		Path genPath = new Path(temp);
		return genPath;
	}

	/**
	This method set's the overall maximum speed for the Path.
	**/

	
	public Path removeToIndex(int i) {
		for(int n = i; n >= 0; n--) {
			this.remove(n);
		}
		return this;
	}
	
	/**
	The setTarVel method is used to set the target velocities at each Point in the 
	Path. 
	**/

	public void setTarVel(){
		//This is for the old target velocities
		double vNot = get(0).getVel();
		double a = 1;
		for(int i = 0; i < size() - 2; i++)
			get(i).setVel(Math.sqrt((Math.pow(vNot, 2) + (2 * a 
					* get(i).distFrom(get(i+1))))));
		
		//actual target velocities
		get(size()-1).setVel(0);
		double distance = 0;
		for(int i = size() - 2; i > -1; i--){
			distance = get(i).distFrom(get(i+1));
			get(i).setVel(Math.min(get(i).getVel(), 
					Math.sqrt(Math.pow(get(i+1).getVel() , 2) + (2 * a * distance))));
		}
	}

	/**
	The generatePath method uses the numPointForArray method to determine the amount
	of points should go into a line segment based on the distance between each point
	**/
	
	public int[] numPointForArray(double dist) {
		int[] numPoints = new int[size()-1];
		for(int i = 0; i < numPoints.length; i++)
			numPoints[i] = (int)((get(i).distFrom(get(i+1)))/dist)-1;
		return numPoints;
	}
	
	public static Path copyPath(Path yuh) {
		Path path = new Path();
		
		for(int i = 0; i < yuh.size(); i++) {
			path.add(yuh.get(i));
		}
		
		return path;
	}
	
	/**
	The generate path class is the class that injects points into the path. to do 
	this,it takes the x and y dimensions of the line segment, divides that number by 
	the number of points you want to be in that segment + 1(because the last point
	will always be the end of the line segment, to get four points into a line 
	segment you must divide it by 5) to get the distance each point will be away 
	from each other. Then, it adds the first point, injects the points, and at the 
	end of the algorithm, adds the last point.
	
	The function of the double for loop is this, the first for loop will loop through 
	each line segment, and the second for loop is used to inject the points onto the 
	path.
	**/
	
	public ArrayList<Point> generatePath(int[] numPoints){
		ArrayList<Point> genPath = new ArrayList<Point>();
		double dimensionX = 0; 
		double dimensionY = 0; 
		double distanceX = 0; 
		double distanceY = 0;
		Point temp = new Point(0,0);
		
		for(int i = 0; i <= size() - 2; i++)
		{
			dimensionX = get(i + 1).getX() - get(i).getX();
			dimensionY = get(i + 1).getY() - get(i).getY();
			
			distanceX = dimensionX / (numPoints[i] + 1);
			distanceY = dimensionY / (numPoints[i] + 1);
			
			temp = new Point(get(i).getX(), get(i).getY());
			genPath.add(new Point(temp.getX(), temp.getY()));

			for(int x = 0; x < numPoints[i]; x++)
			{
				temp.setX(temp.getX() + distanceX);
				temp.setY(temp.getY() + distanceY);
				genPath.add(new Point(temp.getX(), temp.getY()));
			}
		}
		genPath.add(new Point(get(size()-1).getX(), get(size()-1).getY()));
		return genPath;
	}

	/**
	This method calculates the curve.
	It takes a path, and parameters a, b, and tolerance. This algorithm is borrowed from Team 2168, and it is
	recommended that b be within .75 and .98, with a set to 1 - b, and tolerance = 0.001.
	**/

	public static double[][] smoother(double[][] path, double a, double b, double tolerance) {
		double[][] genPath = new double[path.length][path[0].length];
		for(int r = 0; r < path.length; r++)
			for(int c = 0; c < path[0].length; c++)
				genPath[r][c] = path[r][c];
		double change = tolerance;
		
		while(change >= tolerance){
			change = 0.0;
			for(int row = 1; row < path.length - 1; row++){
				for(int col = 0; col < path[row].length; col++){
					double temp = genPath[row][col];
					genPath[row][col] += a * (path[row][col] - genPath[row][col]) 
							+ b * (genPath[row - 1][col] + genPath[row + 1][col] 
									- (2.0 * genPath[row][col]));
					change += Math.abs(temp - genPath[row][col]);
				}
			}
		}
		return genPath;
	}

	/**
	The modified smoother class is an instance class that was created specifically so that instead of having
	to transition all the Path objects to double[][] in the main method, we could call this method to do it for us.
	**/

	public Path smoother(double a, double b, double tolerance){
		return new Path(matrixToPath(smoother(this.pathToMatrix(), a, b,tolerance)));
	}

	/**
	The formula here uses a systems of equations format to find the radius of the 
	circle, then to get the curvature of the circle. The purpose of it is to find 
	the curvature of the turn the robot wants to take, so that it can modulate its 
	speed based on the curvature of the turn. 

	Usually, the parameters should be the point you want to turn at, and the points 
	on either side of it, where Q is on the leftmost of the turn, R is the rightmost,
	and P is the desired point of curvature. 

	Some notes, if the result is NaN, that means the curvature is zero and the 
	radius is infinite, so therefore the path is a straight. Also, if x1 is equal 
	to x2, then you get a divide by zero error. To fix this, add a small value to x1,
	such as 0.001, and the issue will be fixed with minimal error.
	**/
	
	public static double curvatureOfPath(Point P, Point Q, Point R){
		double xOne = P.getX();
		double xTwo = Q.getX();
		double xThree = R.getX();
		double yOne = P.getY();
		double yTwo = Q.getY();
		double yThree = R.getY();
		double kOne = 0.5 * (Math.pow(xOne, 2) + Math.pow(yOne, 2) - Math.pow(xTwo, 2) 
				- Math.pow(yTwo, 2) / (xOne-xTwo));
		double kTwo = (yOne -yTwo) / (xOne-xTwo);
		double b = 0.5 * (Math.pow(xTwo, 2) - 2 * xTwo * kOne * Math.pow(yTwo, 2)
				- Math.pow(xThree, 2) + 2  * xThree * kOne - Math.pow(yThree, 2))
				/ (xThree * kTwo - yThree + yTwo - xTwo * kTwo);
		double a = kOne - kTwo * b;
		double r = Math.sqrt(Math.pow((xOne - a), 2)  + Math.pow((yOne - b), 2));
		double curvature = 1 / r;
		return curvature;
	}
	
	/**
	The constrain method takes a value as well as a minimum and a maximum and 
	constrains the value to be within the range.
	**/
	
	public static double constrain(double num, double min, double max) {
		if(num <= max && num >= min) {
			return num;
		} else if(num > max) {
			return max;
		} else if(num < min) {
			return min;
		}
		return 0;
	}
	
    public Point closestPoint(Point position, Point cPoint) {
        
        Point closest = cPoint;
        int i;
        
        for(i = 1; i < size(); i++) {
            if(position.distFrom(get(i)) < position.distFrom(closest)) {
                closest = get(i);
                return closest;
            }
        }
        
        return closest;
    }

	
	/**
	The lookAhead method uses quadratic equation to find intersect points, then 
	passes the x(AKA the roots) values on to the lookAheadPoint class.
	**/
	
	public static double lookAhead(Point E,  Point L, Point C, double r) {
		Vector2d d = new Vector(L,E);
		Vector2d f = new Vector(E,C);
		
		double a = d.dot(d);
		double b = 2 * f.dot(d);
		double c = f.dot(f) - r*r ;
		
		double discriminant = ((Math.pow(b,2)) - (4*a*c));
		
		if( discriminant < 0 ) {
			return -1;
		}
		
		discriminant = (double)Math.sqrt(discriminant);
		
		double x1 = (-b - discriminant) / (2*a);
		double x2 = (-b + discriminant) / (2*a);
		
		if(x1 >= 0 && x1 <= 1) {
		    return x1;
		}
		
		if(x2 >= 0 && x2 <= 1) {
			return x2;
		}
		
		return -1;
	}

	public static Point findLookAheadPoint(Path path, double L, Point cPosition, Point lPoint) {
		double t = 0;
		double fIndex = 0;
		int i = 0;
		for(i = 0; i < path.size()-1; i++) {
			t = lookAhead(path.get(i), path.get(i+1), cPosition, L);
			fIndex = t + i;
			if(t >= 0 && t <= 1) {
				if(fIndex > lPoint.fIndex()) {
					lPoint.setIndex(fIndex);
					return lPoint = new Point(path.get(i).getX(), path.get(i).getY());
				}
			}
		}
		return lPoint;
	}
}