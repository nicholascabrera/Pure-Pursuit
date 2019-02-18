/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6911.robot;

import org.usfirst.frc.team6911.robot.navigation.Path;
import org.usfirst.frc.team6911.robot.navigation.Point;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
	private double fIndex;					//fractional index, used in lookAhead
	private Point lPoint = new Point(0,0);	//look ahead point
	private Point currentPosition;
	private double xLocation;
	private double yLocation;
	private double distance;
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE CONTROL LOOP VARIABLES		                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private double lDistance;		//look ahead distance
	private double V;				//target robot velocity
	private double L;				//target Left wheels speed
	private double R;				//target right wheels speed
	private double C;				//curvature of arc
	private double T = 26.296875;	//track width
	private double tAccel;			//target acceleration
	
	private double kA = 0.00;//2;	//acceleration constant
	private double kP = 0.0;//1;	//proportional feedback constant
	private double kV = 3.3;		//velocity constant
	
	private double ffL;				//Left feed forward term
	private double fbL;				//Left feedback term
	private double ffR;				//Right feed forward term
	private double fbR;				//Right feedback term
	
	
	
	
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
		
		/*----------------------------------------------------------------------*/
		/*																		*/
		/* SETTING UP PATH AND SMOOTHER CLASS				                    */
		/*																		*/
		/*----------------------------------------------------------------------*/
		
		double weight_smooth = 0.80;
		double a = 1 - weight_smooth;
		double tolerance = 0.001;
		
		Path path = (new Path(new Point[] {new Point(1,1,0), new Point(129.54,1), 
				new Point(129.54,-86.625), new Point(142.27, -99.355)}));
		
		Path genPath = new Path(path.generatePath(path.numPointForArray(6)));
		
		genPath = genPath.smoother( a, weight_smooth, tolerance);
		genPath.setTarVel();

		/*----------------------------------------------------------------------*/
		/*																		*/
		/* GETTING LOCATION FOR THE ROBOT					                    */
		/*																		*/
		/*----------------------------------------------------------------------*/
		
		distance = Math.abs((6*3.14)*(rEncoder.get() + lEncoder.get()/2)/360);
		xLocation = distance * Math.cos(g.getAngle());
		yLocation = distance * Math.sin(g.getAngle());
		currentPosition = new Point(xLocation, yLocation);
		
		//SEPARATION
		
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
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* CONTROL LOOP										                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	public void controller(Path genPath) {
		
		while(genPath.size() > 1) {
			
			C = curvature(lDistance);
			
			genPath = Path.copyPath(genPath.closestPoint(currentPosition));
			
			V = rateLimiter(genPath.get(0).getVel());
			
			tAccel = (T * V) / 2;
			
			L = (V * (2 + (C * T))) / 2;
			R = (V * (2 - (C * T))) / 2;
			
			ffL = kV * L + kA * tAccel;
			ffR = kV * R + kA * tAccel;
			fbL = kP * (L - getSpeed());
			fbR = kP * (R - getSpeed());
			//Code to give each side of the drive train is the (FF + FB)
			
		}
		
	}
	
	public double getSpeed() {
		return 99;
	}
	
	public void findLookAheadPoint(Path pa, double L) {
		double t = 0;
		double fIndex = 0;
		int i = 0;
		for(i = 0; i < pa.size()-1; i++) {
			t = Path.lookAhead(pa.get(i), pa.get(i+1), currentPosition, L);
			if(t >= 0 && t <= 1) {
				if(fIndex > this.fIndex) {
					this.fIndex = fIndex;
					this.lPoint = new Point(pa.get(i).getX(), pa.get(i).getY());
				} else {
				}
			}
		}
	}
	
	public double curvature(double L) {
		
		//variable instantiation
		double localX = this.currentPosition.getX();
		double localY = this.currentPosition.getY();
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
	
	public double rateLimiter(double input) {
		double deltaT = t.get() - this.time;
		this.time = t.get();
		double maxChange = deltaT * maxRate;
		output += Path.constrain(input - output, -maxChange, maxChange);
		return output;
	}
}