/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6911.robot;

import org.usfirst.frc.team6911.robot.navigation.Controller;
import org.usfirst.frc.team6911.robot.navigation.Point;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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

		double weight_smooth = 0.80;
		double tolerance = 0.001;
		
		Controller comp = new Controller(new Point[] {new Point(1,1,0), new Point(129.54,1), 
				new Point(129.54,-86.625), new Point(142.27, -99.355)}, weight_smooth, tolerance);
		
		
	
		comp.controlLoop();
		
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
		
		if(j.getRawButton(1)) {
			s1.set(DoubleSolenoid.Value.kForward);
		}
		
		if(j.getRawButton(2)) {
			s1.set(DoubleSolenoid.Value.kReverse);
		}
	}
	
	@Override
	public void testPeriodic() {}
	

}