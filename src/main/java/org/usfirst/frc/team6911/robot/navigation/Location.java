package org.usfirst.frc.team6911.robot.navigation;

public class Location {
	private Point currentPosition;
	public Location(Point currentPosition) {
		super();
		this.currentPosition = currentPosition;
	}

	public Point getCurrentPosition() {
		return currentPosition;
	}

	public void setCurrentPosition(Point currentPosition) {
		this.currentPosition = currentPosition;
	}
	
	public void move(double changeLeft, double changeRight, double angle) {
		double distance = (changeLeft + changeRight)/2.0;
		
		double newX = distance * Math.cos(Math.toRadians(angle));
		double newY = distance * Math.sin(Math.toRadians(angle));
		//currentPosition.moveX(newX - currentPosition.getX());
		//currentPosition.moveY(newY - currentPosition.getY());
	}
	
}
