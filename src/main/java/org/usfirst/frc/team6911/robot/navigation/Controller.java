package org.usfirst.frc.team6911.robot.navigation;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Controller {
    
    
    /*----------------------------------------------------------------------------*/
    /*                                                                       	 */
    /* INSTANTIATION OF THE PATH OBJECT      	                	             */
    /*                                                                       	 */
    /*----------------------------------------------------------------------------*/

    Path genPath;
    
    /*----------------------------------------------------------------------------*/
    /*                                                                       	 */
    /* INSTANTIATION OF THE LOOK AHEAD VARAIBLES                 	             */
    /*                                                                       	 */
    /*----------------------------------------------------------------------------*/
    

    private Point lPoint = new Point(0,0);  //look ahead point
    private double lDistance;               //look ahead distance
    private Point currentPosition;
    private Point cPoint = new Point(0,0);  //closest point
    

    /*----------------------------------------------------------------------------*/
    /*                                                                       	 */
    /* INSTANTIATION OF THE RATE LIMITER VARIABLES   	                         */
    /*                                                                       	 */
    /*----------------------------------------------------------------------------*/
    

    private double xLocation;
    private double yLocation;
    private double distance;
    private double time;            //time for time difference in rate limiter
    private double output;          //rate limiter output
    private double maxRate;         //maximum rate of acceleration - rate limiter
    
    
    /*----------------------------------------------------------------------------*/
    /*                                                                       	 */
    /* INSTANTIATION OF THE CONTROL LOOP VARIABLES   	                         */
    /*                                                                       	 */
    /*----------------------------------------------------------------------------*/
    
    private double V;               //target robot velocity
    private double LO;              //target Left wheels speed
    private double RO;              //target right wheels speed
    private double LF;              //target Left wheels speed
    private double RF;              //target right wheels speed
    private double C;               //curvature of arc
    private double T = 26.296875;   //track width
    private double tAccel;          //target acceleration
    
    private double kA = 0.00;//2;   //acceleration constant
    private double kP = 0.0;//1;    //proportional feedback constant
    private double kV = 3.3;        //velocity constant
    
    private double ffL;             //Left feed forward term
    private double fbL;             //Left feedback term
    private double ffR;             //Right feed forward term
    private double fbR;             //Right feedback term
    private Double Left;
    private Double Right;
    
    private HashMap<String, Double> wV = new HashMap<>();
    public boolean isFinished = false;
    
    
    
    public Controller(Point[] path, double weight_smooth, double tol) {
        
        double a = 1 - weight_smooth;
        
        Path p = new Path(path);
        
        int[] numPoints = p.numPointForArray(6);
        
        this.genPath = new Path(p.generatePath(numPoints));
        
        genPath = genPath.smoother( a, weight_smooth, tol);
        genPath.setTarVel();
    }
    
    
    
	public HashMap<String, Double> controlLoop(double lPosition, double rPosition, double heading, double lSpeed, 
		double rSpeed, double cTime) {
        
        this.distance = Math.abs((6*3.14)*(rPosition + lPosition/2)/360);
        this.xLocation = this.distance * Math.cos(heading);
        this.yLocation = this.distance * Math.sin(heading);
        currentPosition = new Point(this.xLocation, this.yLocation);
        
        while(genPath.size() > 1 || this.isFinished != false) {
            
            this.lPoint = Path.findLookAheadPoint(genPath, this.lDistance, currentPosition, this.lPoint);

            this.C = Point.curvature(this.lDistance, currentPosition, heading, this.lPoint);

            this.cPoint = genPath.closestPoint(currentPosition, this.cPoint);
			
            this.V = this.cPoint.getVel();
            
            this.LF = (this.V * (2 + (this.C * this.T))) / 2;
            this.RF = (this.V * (2 - (this.C * this.T))) / 2;
            
            double dist = currentPosition.distFrom(this.lPoint);

            this.tAccel = ( ( (this.LF * this.LF) - (this.LO * this.LO) ) / (2 * dist) );
            
            this.ffL = this.kV * this.LF + this.kA * this.tAccel;
            this.ffR = this.kV * this.RF + this.kA * this.tAccel;
            this.fbL = this.kP * (this.LF - lSpeed);
            this.fbR = this.kP * (this.RF - rSpeed);
            
            this.Left = (this.ffL + this.fbL);
            this.Right = (this.ffR + this.fbR);

            this.LO = this.LF;
            this.RO = this.RF;
            
            this.wV.put("Left", Left);
            this.wV.put("Right", Right);
            
            return this.wV;
        }
        
        this.isFinished = true;
        return this.wV;
    }

    public double rateLimiter(double input, double cTime) {
        double deltaT = cTime - this.time;
        this.time = cTime;
        double maxChange = deltaT * maxRate;
        output += Path.constrain(input - output, -maxChange, maxChange);
        return output;
    }
}
