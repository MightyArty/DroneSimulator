import java.util.Random;

public class Tools {
	public static Point getPointByDistance(Point fromPoint, double rotation, double distance) {
		double radians = Math.PI*(rotation/180);
		
		double i= distance/ GlobalParams.CMPerPixel;
		double xi = fromPoint.x + Math.cos(radians)*i;
		double yi = fromPoint.y + Math.sin(radians)*i;
		return new Point(xi,yi);
	}

	public static double noiseBetween(double min,double max,boolean isNegative) {
		Random rand = new Random();
		double noiseToDistance = 1;
		double noise = (min + rand.nextFloat()*(max-min))/100;
		if(!isNegative) {
			return noiseToDistance + noise;
		}
		if(rand.nextBoolean()) {
			return noiseToDistance + noise;
		} else {
			return noiseToDistance - noise;
		}
	}

	public static double getDistanceBetweenPoints(Point from, Point to) {
		double x1 = (from.x - to.x)*(from.x - to.x);
		double y1 = (from.y-to.y)*(from.y-to.y);
		return Math.sqrt(x1+y1);
	}
}
