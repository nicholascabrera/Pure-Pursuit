package org.usfirst.frc.team6911.robot.navigation;

import java.util.ArrayList;

@SuppressWarnings("serial")
public class Path extends ArrayList<Point>{
	
	public Path() {
		
	}
	public Path(Iterable<Point> path) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
	}
	
	public Path(Point[] path) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
	}
	
	public int[] numPointForArray(double dist) {
		int[] numPoints = new int[size()-1];
		for(int i = 0; i < numPoints.length; i++)
			numPoints[i] = (int)((get(i).distFrom(get(i+1)))/dist)-1;
		return numPoints;
	}
	
	//generatePath class
	public ArrayList<Point> generatePath(int[] numPoints)
	{
		ArrayList<Point> genPath = new ArrayList<Point>();
		double dimensionX = 0; double dimensionY = 0; double distanceX = 0; double distanceY = 0;
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
	This code will help with going from path to matrix, and back, as the code for smoothing 
	the path needs a matrix instead of the current ArrayList of Points
	**/

	/**
	What is happening in this code is instantiation, then a for loop where it loops through 
	each element of the ArrayList, and puts the X value for each point as the first column, 
	and the Y value for each point as the second column
	**/

	public double[][] pathToMatrix(ArrayList<Point> path){
		double[][] genPath = new double[path.size()][1];
		for(int i = 0; i < path.size(); i++){
			genPath[i][0] = path.get(i).getX();
			genPath[i][1] = path.get(i).getX();
			}
		return genPath;
		}

	/**
	What is happening in this code is instantiation, then a for loop where it loops through 
	each row, and where the first column is the X value, and the second column is the Y value,
	 adds a new Point with those values of X and Y
	**/

	public ArrayList<Point> matrixToPath(double[][] path){
		ArrayList<Point> genPath =  new ArrayList<Point>();
		for(int i = 0; i < path[0].length; i++){
			genPath.add(new Point(path[i][0],path[i][1]));
			}
		return genPath;
		}

	public double[][] smoother(double[][] path, double a, double b, double tolerance){
		double[][] genPath = path;
		double change = tolerance;
		
		while(change >= tolerance){
			change = 0.0;
			for(int row = 0;row < path.length - 1; row++){
				for(int col = 0;col < path[0].length - 1; col++){
					double temp = genPath[row][col];
					genPath[row][col] += a * (path[row][col] - genPath[row][col]) + b * (genPath[row - 1][col] + genPath[row + 1][col] - (2.0 * genPath[row][col]));
					change += Math.abs(temp - genPath[row][col]);
					}
				}
			}
		return genPath;
		}
	
	/**
	I’m not quite sure how this formula works, so I cant really explain it. However, the 
	purpose of it is to find the curvature of the turn the robot wants to take, so that it 
	can modulate its speed based on the curvature of the turn. 

	Usually, the parameters should be the point you want to turn at, and the points on either 
	side of it, where Q is on the leftmost of the turn, R is the rightmost, and P is the 
	desired point of curvature. 

	Some notes, if the result is NaN, that means the curvature is zero and the radius is 
	infinite, so therefore the path is a straight. Also, if x1 is equal to x2, then you get a 
	divide by zero error. To fix this, add a small value to x1, such as 0.001, and the issue 
	will be fixed with minimal error.
	**/
	
	public double curvatureOfPath(Point P, Point Q, Point R){
		double xOne = P.getX();
		double xTwo = Q.getX();
		double xThree = R.getX();
		double yOne = P.getY();
		double yTwo = Q.getY();
		double yThree = R.getY();
		double kOne = 0.5 * (Math.pow(xOne, 2) + Math.pow(yOne, 2) - Math.pow(xTwo, 2) - Math.pow(yTwo, 2git) / (xOne-xTwo));
		double kTwo = (yOne -yTwo) / (xOne-xTwo);
		double b = 0.5 * (Math.pow(xTwo, 2) - 2 * xTwo * kOne * Math.pow(yTwo, 2)  - Math.pow(xThree, 2) + 2  * xThree * kOne - Math.pow(yThree, 2)) / (xThree * kTwo - yThree + yTwo - xTwo * kTwo);
		double a = kOne - kTwo * b;
		double r = Math.sqrt(Math.pow((xOne - a), 2)  + Math.pow((yOne - b), 2));
		double curvature = 1 / r;
		return curvature;
	}

}

