import java.awt.Graphics;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class Drone {
	private Point sensorOpticalFlow;
	private Point pointFromStart;
	private CPU cpu;

	private double gyroRotation;
	private double rotation;
	private double speed;
	
	private int pitch;
	private int roll;
	private double high;
	private double ceiling;
	
	public static boolean is_landing = true;
	public static boolean toogleArm = true;
	
	public Point startPoint;
	public List<Lidar> lidars;
	public Map realMap;
	
	public Drone(Map realMap) {
		this.realMap = realMap;
		this.startPoint = realMap.drone_start_point;
		pointFromStart = new Point();
		sensorOpticalFlow = new Point();
		lidars = new ArrayList<>();
		speed = 0.2;
		this.pitch = 0;
		this.roll = 0;
		this.high = 100;
		this.ceiling = GlobalParams.max_ceiling - this.high;
		rotation = 0;
		gyroRotation = rotation;
		cpu = new CPU(100,"Drone");
	}
	
	public void play() {
		cpu.play();
	}

	public void addLidar(int degrees) {
		Lidar lidar = new Lidar(this,degrees);
		lidars.add(lidar);
		cpu.addFunction(lidar::getSimulationDistance);
	}
	
	public Point getPointOnMap() {
		double x = startPoint.x + pointFromStart.x;
		double y = startPoint.y + pointFromStart.y;
		return new Point(x,y);
	}
	
	public void update(int deltaTime) {
		double noiseToDistance = Tools.noiseBetween(GlobalParams.min_motion_accuracy, GlobalParams.max_motion_accuracy, false);
		double noiseToRotation = Tools.noiseBetween(GlobalParams.min_rotation_accuracy, GlobalParams.max_rotation_accuracy, false);
		double milli_per_minute = 60000;
		if (toogleArm)
		{
			double distancedMoved = (speed*100)*((double)deltaTime/1000);
			pointFromStart =  Tools.getPointByDistance(pointFromStart, rotation, distancedMoved);
			sensorOpticalFlow = Tools.getPointByDistance(sensorOpticalFlow, rotation, distancedMoved*noiseToDistance);
			gyroRotation += (1-noiseToRotation)*deltaTime/milli_per_minute;
			gyroRotation = formatRotation(gyroRotation);
			this.high += (1-noiseToDistance)*deltaTime/milli_per_minute;
			this.ceiling += (1-noiseToDistance)*deltaTime/milli_per_minute;
		}
		else
		{
			double distancedMoved = (speed*100)*((double)deltaTime/1000);
			
			if (is_landing)
			{
				this.high -= distancedMoved*noiseToDistance;
				this.ceiling += distancedMoved*noiseToDistance;
				if (this.high<0)
				{
					this.high = 0;
					this.ceiling = GlobalParams.max_ceiling - distancedMoved*noiseToDistance;
					this.pitch = 0;
					this.slowDown(0);
				}
			}
			else
			{
				this.high += distancedMoved*noiseToDistance;
				this.ceiling -= distancedMoved*noiseToDistance;
				if (this.high>99)
				{
					this.high = 100;
					this.ceiling = GlobalParams.max_ceiling - this.high + distancedMoved*noiseToDistance;
					is_landing = true;
					toogleArm = !toogleArm;
					AutoAlgo1.is_flight = true;
					AutoAlgo1.is_finish = false;
				}
			}
		}
	}
	
	public static void ArmDisarm()
	{
		if (toogleArm) {
			toogleArm = !toogleArm;
			AutoAlgo1.is_flight = false;
			AutoAlgo1.is_finish = true;
		}
		else
		{
			is_landing = false;
		}
	}
	public static double formatRotation(double rotationValue)
	{
		rotationValue %= 360;
		if(rotationValue < 0) {
			rotationValue = 360 -rotationValue;
		}
		return rotationValue;
	}
	
	public double getRotation() {
		return rotation;
	}
	
	public double getGyroRotation() {
		return gyroRotation;
	}
	
	public Point getOpticalSensorLocation() {
		return new Point(sensorOpticalFlow);
	}

	public void rollLeft() {
		this.roll = -10;
	}
	
	public void rollRight() {
		this.roll = 10;
	}
	
	public void rotateLeft(int deltaTime) {
		double rotationChanged = GlobalParams.rotation_per_second*deltaTime/1000;
		
		rotation += rotationChanged;
		rotation = formatRotation(rotation);
		
		gyroRotation += rotationChanged;
		gyroRotation = formatRotation(gyroRotation);
	}
	
	public void speedUp(int deltaTime) {
		this.pitch = 10;
		speed += (GlobalParams.accelerate_per_second*deltaTime/1000);
		if(speed > GlobalParams.max_speed) {
			speed = GlobalParams.max_speed;
		}
	}
	
	public void slowDown(int deltaTime) {
		this.pitch = 0;
		this.roll = 0;
		speed -= (GlobalParams.accelerate_per_second*deltaTime/1000);
		if(speed < 0) {
			speed = 0;
		}
	}
	boolean initPaint = false;
	public void paint(Graphics g) {
		if(!initPaint) {
			try {
				initPaint = true;
			} catch(Exception ex) {
			}
		}
		for(int i=0;i<lidars.size();i++) {
			Lidar lidar = lidars.get(i);
			lidar.paint(g);
		}
	}
	
	public String getInfoHTML() {
		DecimalFormat df = new DecimalFormat("#.####");
		String info = "<html>";
		info += "<span>Ceiling: <span style=\"color:blue;font-weight:bold\">" + df.format(this.ceiling) +" cm</span></span><br>";
		info += "<span>Baro: <span style=\"color:blue;font-weight:bold\">" + df.format(this.high) +" cm</span></span><br>";
		info += "<span>Pitch Axis: <span style=\"color:blue;font-weight:bold\">" + this.pitch +"&deg;</span></span><br>";
		info += "<span>Roll Axis: <span style=\"color:blue;font-weight:bold\">" + this.roll +"&deg;</span></span><br>";
		info += "<span>Yaw Axis: <span style=\"color:blue;font-weight:bold\">" + df.format(gyroRotation) +"&deg;</span></span><br>";
		info += "</html>";
		return info;
	}
}
