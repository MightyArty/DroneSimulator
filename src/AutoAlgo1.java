import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.Random;

public class AutoAlgo1 {

	int map_size = 3000;
	enum PixelState {blocked,explored,unexplored,visited};
	PixelState map[][];
	Drone drone;
	Point droneStartingPoint;
	ArrayList<Point> points;
	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;
	boolean isSpeedUp = false;
	Graph mGraph = new Graph();
	CPU ai_cpu;
	public AutoAlgo1(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func =  new ArrayList<>();
		points = new ArrayList<Point>();
		drone = new Drone(realMap);
		drone.addLidar(0);
		drone.addLidar(90);
		drone.addLidar(-90);
		drone.addLidar(-180);
		initMap();
		isRotating = 0;
		ai_cpu = new CPU(200,"Auto_AI");
		ai_cpu.addFunction(this::update);
	}

	public void initMap() {
		map = new PixelState[map_size][map_size];
		for(int i=0;i<map_size;i++) {
			for(int j=0;j<map_size;j++) {
				map[i][j] = PixelState.unexplored;
			}
		}
		droneStartingPoint = new Point(map_size/2,map_size/2);
	}

	public void play() {
		drone.play();
		ai_cpu.play();
	}


	public void update(int deltaTime) {
		updateVisited();
		updateMapByLidars();

		moving_algorithm(deltaTime);

		if (!is_finish) {
			if(isRotating != 0) {
				updateRotating(deltaTime);
			}

			if(isSpeedUp) {
				drone.speedUp(deltaTime);
			} else {
				drone.slowDown(deltaTime);
			}
		}
	}

	public void speedUp() {
		isSpeedUp = true;
	}

	public void speedDown() {
		isSpeedUp = false;
	}

	public void updateMapByLidars() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		for(int i=0;i<drone.lidars.size();i++) {
			Lidar lidar = drone.lidars.get(i);
			double rotation = drone.getGyroRotation() + lidar.degrees;
			for (int distanceInCM=0 ; distanceInCM < lidar.current_distance ; distanceInCM++) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
				setPixel(p.x,p.y,PixelState.explored);
			}
			if(lidar.current_distance > 0 && lidar.current_distance < GlobalParams.lidarLimit - GlobalParams.lidarNoise) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
				setPixel(p.x,p.y,PixelState.blocked);
			}
		}
	}

	public void updateVisited() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		setPixel(fromPoint.x,fromPoint.y,PixelState.visited);
	}

	public void setPixel(double x, double y,PixelState state) {
		int xi = (int)x;
		int yi = (int)y;
		if(state == PixelState.visited) {
			map[xi][yi] = state;
			return;
		}
		if(map[xi][yi] == PixelState.unexplored) {
			map[xi][yi] = state;
		}
	}

	public void paintBlindMap(Graphics g) {
		Color c = g.getColor();
		int i = (int)droneStartingPoint.y - (int)drone.startPoint.x;
		int startY = i;
		for(;i<map_size;i++) {
			int j = (int)droneStartingPoint.x - (int)drone.startPoint.y;
			int startX = j;
			for(;j<map_size;j++) {
				if(map[i][j] != PixelState.unexplored)  {
					if(map[i][j] == PixelState.blocked) {
						g.setColor(Color.RED);
					}
					else if(map[i][j] == PixelState.explored) {
						g.setColor(Color.YELLOW);
					}
					else if(map[i][j] == PixelState.visited) {
						g.setColor(Color.BLUE);
					}
					g.drawLine(i-startY, j-startX, i-startY, j-startX);
				}
			}
		}
		g.setColor(c);
	}

	public void paintPoints(Graphics g) {
		for(int i=0;i<points.size();i++) {
			Point p = points.get(i);
			g.drawOval((int)p.x + (int)drone.startPoint.x - 10, (int)p.y + (int)drone.startPoint.y-10, 20, 20);
		}

	}

	public void paint(Graphics g) {
		if(SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}

		paintBlindMap(g);
		paintPoints(g);

		drone.paint(g);
	}

	public boolean checkCollision(double front, double right, double left, double back) {
		// in gray zone
		if (front < GlobalParams.risky_distance && right < GlobalParams.risky_distance &&
				left < GlobalParams.risky_distance && back < GlobalParams.risky_distance) {
			return true;
		}
		// front collision
		if (front < GlobalParams.risky_distance && front > 0 ) {
			return true;
		}
		// right collision
		if (right < GlobalParams.risky_distance && right > 0 ) {
			return true;
		}
		// left collision
		if (left < GlobalParams.risky_distance && left > 0 ) {
			return true;
		}
		// back collision
		if (back < GlobalParams.risky_distance && back > 0 ) {
			return true;
		}
		return false;
	}

	public float getAngle(Point dronePoint) {
		Point target = getLastPoint();
	    float angle = (float) Math.toDegrees(Math.atan2(target.y - dronePoint.y, target.x - dronePoint.x));
	    if (angle < 0) {
	        angle += 360;
	    }
	    return angle;
	}

	static boolean is_finish = false;
	int max_risky_distance = 150;
	int max_angle_risky = 10;
	double max_distance_between_points = 100;
	boolean is_init = true;
	boolean try_to_escape = false;
	boolean is_risky = false;
	boolean stop_rotate = false;
	boolean point_saved = false;
	public static boolean is_flight = false;
	public static boolean calculate_angle = false;
	Point init_point;
	Point stop_rotate_point;
	int stop_rotate_distance = 0;
	int left_right = 0;

	public void moving_algorithm(int deltaTime) {
		point_saved = false;
		if(is_init) {
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			points.add(dronePoint);
			mGraph.addVertex(dronePoint);
			is_init = false;
			point_saved = true;

			Lidar right = drone.lidars.get(1);
			if(right.getSimulationDistance(deltaTime) > max_risky_distance / 3 ) {
				this.spinBy(90, true, new Func() {
					@Override
					public void method() {
						try_to_escape = false;
						is_risky = false;
						is_flight = true;
						speedUp();
					}
				});
			}
		}
		if (is_flight) {
			Point dronePoint = drone.getOpticalSensorLocation();
			if (stop_rotate) {
				double distance = Tools.getDistanceBetweenPoints(dronePoint, stop_rotate_point);
				if (distance>stop_rotate_distance) {
					stop_rotate = false;
					stop_rotate_point = null;
				}
				return;
			}
			double front_distance = drone.lidars.get(0).getSimulationDistance(deltaTime);
			double right_distance = drone.lidars.get(1).getSimulationDistance(deltaTime);
			double left_distance = drone.lidars.get(2).getSimulationDistance(deltaTime);
			double back_distance = drone.lidars.get(3).getSimulationDistance(deltaTime);
			if(!is_risky) {
				if(front_distance <= max_risky_distance || right_distance <= max_risky_distance/3 || left_distance <= max_risky_distance/3)
				{
					is_risky = true;
					if (checkCollision(front_distance, right_distance, left_distance, back_distance))
					{
						speedDown();
						flightStop();
						SimulationWindow.disabledAllButtons();
						SimulationWindow.reason_to_stop = SimulationWindow.crash_reason;
					}
				}
			}
			else
			{
				if(SimulationWindow.return_home)
				{
					if(Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) <  max_distance_between_points/6)
					{
						removeLastPoint();
						if (points.size() == 0) {
							speedDown();
							SimulationWindow.disabledAllButtons();
							SimulationWindow.reason_to_stop = SimulationWindow.target_reason;
							Drone.ArmDisarm();
						}
						calculate_angle = true;
					}
				} else
				{
					if( Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >=  max_distance_between_points)
					{
						points.add(dronePoint);
						mGraph.addVertex(dronePoint);
						point_saved = true;
						if (front_distance > 298 && right_distance > 298 && left_distance > 298 && back_distance > 298) {
							Random ran = new Random();
							int spin_by = ran.nextInt(720) - 360;
							speedDown();
							spinBy(spin_by, false, new Func() {
								@Override
								public void method() {
									try_to_escape = false;
									is_risky = false;
									speedUp();
								}
							});
							return ;
						}
					}
				}
				if(!try_to_escape)
				{
					try_to_escape = true;
					int spin_by = max_angle_risky;
					if(SimulationWindow.return_home)
					{
						if (calculate_angle)
						{
							double angle = getAngle(dronePoint);
							double curr =  drone.getGyroRotation();
							spin_by = (int) (angle - curr);
							calculate_angle = false;
						}
						else
						{
							if (front_distance > 0 && front_distance < max_risky_distance/2)
							{
								double angle = getAngle(dronePoint);
								double curr =  drone.getGyroRotation();
								spin_by = (int) (angle - curr);
							}
							else
							{
								if(left_right == 0)
								{
									spin_by *= (-1);
									left_right = 1;
								}
								else
								{
									left_right = 0;
								}
							}
						}
					}
					else
					{
						if (front_distance > 0 && front_distance < max_risky_distance/2)
						{
							stop_rotate_point = new Point(dronePoint);
							speedDown();
							stop_rotate = true;
							int degree = 0;
							if (left_distance > max_risky_distance)
							{
								left_right = 1;
								degree = -90;
							}
							if (right_distance > max_risky_distance)
							{
								left_right = 0;
								degree = 90;
							}
							if (back_distance > max_risky_distance)
							{
								degree = -180;
							}
							if (!point_saved)
							{
								points.add(dronePoint);
								mGraph.addVertex(dronePoint);
							}
							this.spinBy(degree, false, new Func() {
								@Override
								public void method() {
									try_to_escape = false;
									is_risky = false;
									speedUp();
								}
							});
							return;
						}
						if (right_distance <= max_risky_distance / 3 && right_distance > 0 && left_distance > max_risky_distance / 4)
						{
							left_right = 1;
							spin_by *= (-1);
						}
						else
						{
							if (left_distance <= max_risky_distance/3 && left_distance > 0)
							{
								left_right = 0;
							}
							else
							{
								if(left_right == 0)
								{
									spin_by *= (-1);
									left_right = 1;
								}
								else
								{
									left_right = 0;
								}
							}
						}
					}
					spinBy(spin_by, false, new Func() {
						@Override
						public void method() {
							try_to_escape = false;
							is_risky = false;
						}
					});
				}
			}
		}
	}

	public void flightStart() {
		is_flight = true;
	}

	public void flightStop() {
		is_flight = false;
	}

	public void calculateAngle() {
		calculate_angle = true;
	}

	double lastGyroRotation = 0;
	public void updateRotating(int deltaTime) {
		if(degrees_left.size() == 0) {
			return;
		}
		double degrees_left_to_rotate = degrees_left.get(0);
		boolean isLeft = true;
		if(degrees_left_to_rotate > 0)
		{
			isLeft = false;
		}
		double curr =  drone.getGyroRotation();
		double just_rotated = 0;
		if (isLeft)
		{
			just_rotated = curr - lastGyroRotation;
			if(just_rotated > 0) {
				just_rotated = -(360 - just_rotated);
			}
			drone.rollLeft();
		}
		else
		{
			just_rotated = curr - lastGyroRotation;
			if(just_rotated < 0)
			{
				just_rotated = 360 + just_rotated;
			}
			drone.rollRight();
		}
		lastGyroRotation = curr;
		degrees_left_to_rotate-=just_rotated;
		degrees_left.remove(0);
		degrees_left.add(0,degrees_left_to_rotate);

		if((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
			degrees_left.remove(0);
			Func func = degrees_left_func.get(0);
			if(func != null) {
				func.method();
			}
			degrees_left_func.remove(0);
			if(degrees_left.size() == 0) {
				isRotating = 0;
			}
			return;
		}
		int direction = (int)(degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
		drone.rotateLeft(deltaTime * direction);
	}

	public void spinBy(double degrees,boolean isFirst,Func func) {
		lastGyroRotation = drone.getGyroRotation();
		if(isFirst) {
			degrees_left.add(0,degrees);
			degrees_left_func.add(0,func);
		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(func);
		}
		isRotating = 1;
	}

	public Point getLastPoint() {
		if(points.size() == 0) {
			return init_point;
		}
		Point p1 = points.get(points.size()-1);
		return p1;
	}

	public Point removeLastPoint() {
		if(points.isEmpty()) {
			return init_point;
		}
		return points.remove(points.size()-1);
	}
}
