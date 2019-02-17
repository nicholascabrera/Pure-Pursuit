/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6911.robot;

import org.usfirst.frc.team6911.robot.navigation.Location;
import org.usfirst.frc.team6911.robot.navigation.Path;
import org.usfirst.frc.team6911.robot.navigation.Point;
import org.usfirst.frc.team6911.robot.navigation.Vector;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE PNEUMATIC VARIABLES			                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private DoubleSolenoid s1 = new DoubleSolenoid(0,1);	//pneumatics solenoids
	private Joystick j = new Joystick(0);	//joy stick
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE RATE LIMITER VARIABLES		                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private double time;			//time for time difference in rate limiter
	private double output;			//rate limiter output
	private Timer t = new Timer();	//timer for rate limiter
	private double maxRate;			//maximum rate of acceleration - rate limiter
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE LOCATION AND LOOK AHEAD VARAIBLES                     */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private Encoder lEncoder = new Encoder(0,1),
			rEncoder = new Encoder(2,3);	//encoders for getting location.
	private Gyro g;							//gyroscope for angle
	private Location local = new Location(lEncoder, rEncoder, g);
											//location for look ahead/point
	private double fIndex;					//fractional index, used in lookAhead
	private Point lPoint = new Point(0,0);	//look ahead point
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE CONTROL LOOP VARIABLES		                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private double V;				//target robot velocity
	private double L = 18;			//target Left wheels speed
	private double R;				//target right wheels speed
	private double C;				//curvature of arc
	private double T = 24.296875;	//track width
	private double kA;				//acceleration constant
	private double kP;				//proportional feedback constant
	private double kV;				//velocity constant
	private double FF;				//feed forward term
	private double FB;				//feedback term
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	
	@Override
	public void teleopPeriodic() {
		t.start();
		maxRate = 1;
		
		if(j.getRawButton(1)) {
			s1.set(DoubleSolenoid.Value.kForward);
		}
		
		if(j.getRawButton(2)) {
			s1.set(DoubleSolenoid.Value.kReverse);
		}
	}
	
	@Override
	public void testPeriodic() {}
	
	public Path closestPoint(Path pa) {
		double x, y;
		x = local.getCurrentPosition().getX();
		y = local.getCurrentPosition().getY();
		Point p = new Point(x,y), closest = pa.get(0);
		int i;
		for(i = 1; i < pa.size(); i++) {
			if(p.distFrom(pa.get(i)) < p.distFrom(closest)) {
				closest = pa.get(i);
			}
		}
		
		pa = pa.removeToIndex(i);
		
		return pa;
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
	
	public Point lookAheadPoint(Path pa, double L) {
		double t = 0;
		double fIndex = 0;
		int i = 0;
		for(i = 0; i < pa.size()-1; i++) {
			t = lookAhead(pa.get(i), pa.get(i+1), local.getCurrentPosition(), L);
			if(t >= 0 && t <= 1) {
				if(fIndex > this.fIndex) {
					this.fIndex = fIndex;
					this.lPoint = new Point(pa.get(i).getX(), pa.get(i).getY());
					return this.lPoint;
				} else {
					return this.lPoint;
				}
			}
		}
		return this.lPoint;
	}
	
	public double curvature(double L) {
		
		//variable instantiation
		double localX = this.local.getCurrentPosition().getX();
		double localY = this.local.getCurrentPosition().getY();
		double rAngle = g.getAngle();
		double a = -Math.tan(rAngle);
		double b = 1;
		double c = Math.tan(rAngle) * localX;
		
		//calculations
		double side = Math.signum(Math.sin(rAngle) * (lPoint.getX() - localX) 
				- Math.cos(rAngle) * (lPoint.getY() - localY));
		double x = Math.abs(a * lPoint.getX() + b * lPoint.getY() + c)
				/ Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
		double curvature = (2*x)/(Math.pow(L, 2));
		double sCurvature = side * curvature;
		
		return sCurvature;
	}
	
	public void rateLimiter(double input) {
		double deltaT = t.get() - this.time;
		this.time = t.get();
		double maxChange = deltaT * maxRate;
		output += constrain(input - output, -maxChange, maxChange);
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
}