package org.usfirst.frc.team6911.robot.navigation;

public class Point {
	private double x;
	private double y;
	private double targetV;
	private double maxV;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Point(double x, double y, double tarV) {
		this.x = x;
		this.y = y;
		this.targetV = tarV;
	}
	
	public void setMaxV(double v) {
		maxV = v;
	}
	
	public double getMaxV() {
		return maxV;
	}

	public void setVel(double v){
		targetV = v;
	}

	public double getVel(){
		return targetV;
	}

	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public void setY(double y) {
		this.y = y;
	}
	
	public double angleBetween(Point p) {
		return Math.asin(y-p.getY()/x-p.getX());
	}
	public double distFrom(Point p) {
		return Math.sqrt(Math.abs(p.getX()-x)*Math.abs(p.getX()-x) + Math.abs(p.getY()-y)*Math.abs(p.getY()-y));
	}
	public String toString() {
		return "(" + x + ", " + y + ")";
	}
}
