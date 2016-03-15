package projects.triangulation;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;

/**
 * Created by doms on 1/25/16.
 */
public class TriangleUtils {

    public static Vec2 center(Vec2 a, Vec2 b, Vec2 c) {
        return a.add(b).add(c).mul(1 / 3f);
    }




    // Used by fuzzyContains. Calculates the distance of a point to a line segment
    // Adapted from: http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    public static float squaredDistanceToLineSegment(Vec2 a, Vec2 b, Vec2 p) {
        float l2 = a.sub(b).lengthSquared();
        if (l2 == 0) return p.sub(a).lengthSquared();
        float t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
        if (t < 0) return a.sub(p).lengthSquared();
        if (t > 1) return b.sub(p).lengthSquared();
        return p.sub(new Vec2(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))).lengthSquared();
    }

    /**
     * Returns the intersection points of two circles in order to make a localization with two beacons (usually returns
     * 2 positions). In case the two distances to the beacons are too short, they are automatically enlarged in order to
     * deliver a result with inaccurate sensors. There are always two positions returned (which may coincide) as only with
     * very wrong measurements there are none (in this case an exception is thrown).
     * <p/>
     * This implementation has been adapted from https://gsamaras.wordpress.com/code/determine-where-two-circles-intersect-c/
     *
     * @param c1  The position of beacon 1
     * @param c2  The position of beacon 2 (has to be in the same coordinate system as c1)
     * @param d12 The distance between beacon 1 and 2 (faster if already calculated)
     * @param d1  Distance to beacon 1
     * @param d2  Distance to beacon 2
     * @return Two possible positions in the same coordinate system as c1 and c2
     */
    public static Vec2[] circleIntersection(Vec2 c1, Vec2 c2, float d12, float d1, float d2) {
        if ((d12 + MathUtils.min(d1, d2) < MathUtils.max(d1, d2))) {
            if (1.05f * d12 + MathUtils.min(d1, d2) >= MathUtils.max(d1, d2)) d12 = (MathUtils.max(d1, d2) - MathUtils.min(d1, d2)) * 1.01f;
            else{
               // System.err.println("Localization: Distances invalid: " + d12 + " " + c1 + " " + d1 + " " + c2 + " " + d2);
                return new Vec2[]{null,null};
            }
        }
        assert !c1.equals(c2); //NOT: Circles are identical

        if (d12 >= d1 + d2) { //The distance between the two circles is too big
            //Must be a measurement error -> meet on the line
            return new Vec2[]{c1.sub(c2).mul(d2 / (d1 + d2)).add(c2), c1.sub(c2).mul(d2 / (d1 + d2)).add(c2)};//2x the same
        }

        //There are two intersections
        float a = (d1 * d1 - d2 * d2 + d12 * d12) / (2f * d12);
        float h = MathUtils.sqrt(d1 * d1 - a * a);
        Vec2 p2 = new Vec2(c1.x + (a * (c2.x - c1.x)) / d12, c1.y + (a * (c2.y - c1.y)) / d12);
        return new Vec2[]{new Vec2(p2.x + (h * (c2.y - c1.y)) / d12, p2.y - (h * (c2.x - c1.x)) / d12),
                new Vec2(p2.x - (h * (c2.y - c1.y)) / d12, p2.y + (h * (c2.x - c1.x)) / d12)};
    }
}
