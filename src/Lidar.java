import java.awt.Graphics;
import java.util.Random;

public class Lidar{
	Drone drone;
	double degrees;
	double current_distance = 0;
	
	public Lidar(Drone drone,double degrees) {
		this.drone = drone;
		this.degrees = degrees;
	}
	
	public double getDistance(int deltaTime) {
		Point actualPointToShoot= drone.getPointOnMap();
		double rotation = drone.getRotation()+degrees;
		double distanceInCM = 1;
		while(distanceInCM <= GlobalParams.lidarLimit) {
			Point p = Tools.getPointByDistance(actualPointToShoot, rotation, distanceInCM);
			if(drone.realMap.isCollide((int)p.x,(int)p.y)) {
				break;
			}
			distanceInCM++;
		}
		return distanceInCM;
	}
	
	public double getSimulationDistance(int deltaTime) {
		Random ran= new Random();
		double distanceInCM;
		if(ran.nextFloat() <= 0.05f) {
			distanceInCM = 0;
		} else {
			distanceInCM = getDistance(deltaTime);
			distanceInCM += (int)ran.nextInt(GlobalParams.lidarNoise*2) - GlobalParams.lidarNoise; // +- 5 CM to the final calc
		}
		this.current_distance = distanceInCM;
		return distanceInCM;
	}

	public void paint(Graphics g) {
		Point actualPointToShoot= drone.getPointOnMap();
		double fromRotation = drone.getRotation()+degrees;
		Point to = Tools.getPointByDistance(actualPointToShoot, fromRotation, this.current_distance);
		g.drawLine((int)actualPointToShoot.x,(int)actualPointToShoot.y, (int)to.x, (int)to.y);
	}
}
