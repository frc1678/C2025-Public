package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class PointInPolygon {
	// Checking if a point is inside a polygon
	public static boolean pointInPolygon(Translation2d point, ArrayList<Translation2d> polygon) {
		Path2D path = new Path2D.Double();

		// Move to the first point in the polygon
		path.moveTo(polygon.get(0).getX(), polygon.get(0).getY());

		// Connect the points in the polygon
		for (int i = 1; i < polygon.size(); i++) {
			path.lineTo(polygon.get(i).getX(), polygon.get(i).getY());
		}

		// Close the path
		path.closePath();

		// Create a Point2D object for the test point
		Point2D testPoint = new Point2D.Double(point.getX(), point.getY());

		// Check if the test point is inside the polygon
		return path.contains(testPoint);
	}
}
