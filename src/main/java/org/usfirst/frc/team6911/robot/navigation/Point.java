package org.usfirst.frc.team6911.robot.navigation;

public class Point {
	private double x;
	private double y;
	public Point(double x, double y) {
		this.x = x;
		this.y = y;
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
