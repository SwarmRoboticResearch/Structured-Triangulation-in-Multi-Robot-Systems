package projects.triangulation;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import platypus3000.utils.AngleUtils;
import projects.triangulation.NeighborManager.Neighbor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * This class calculates the best extension point during frontier edge expansion.
 * It also rates triangles for CandidateChecking
 *
 * @author Dominik Krupke
 */
public class ExtensionPointOptimization {
    public static final float MAX_EDGE_LENGTH = 1.05f; //Maximal edge length allowed, triangle exceeding this length are rated 0
    public static final float OPT_EDGE_LENGTH = 0.95f; //Save edge length. Used for the initial point set
    public static final float MAX_RANGE = 1.2f;

    //The edge to expand and its two points
    LocalizationTriangle.Edge frontierEdge;
    Vec2 edge_a;
    Vec2 edge_b;

    //The best position visited during the expansion
    Vec2 best_save_pos = null;
    float best_save_pos_rating = 0;

    //The ratings for the initial edge set
    HashMap<Vec2, Float> ratings = new HashMap<Vec2, Float>(4);
    ArrayList<Vec2> pos_order = new ArrayList<Vec2>();

    public ExtensionPointOptimization(DOTRobotController controller, LocalizationTriangle.Edge edge){
        //Save the frontier edge and the two corresponding points
        this.frontierEdge = edge;
        this.edge_a = edge.triangle.getPosOfTriangleRobot(edge.firstRobot);
        this.edge_b = edge.triangle.getPosOfTriangleRobot(edge.secondRobot);

        //Initial extension point set
        Vec2[] tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), OPT_EDGE_LENGTH, OPT_EDGE_LENGTH);
        Vec2 v0 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]); //The point farthest away from triangle is on the right side
        ratings.put(v0, Float.MAX_VALUE);
        pos_order.add(v0);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), OPT_EDGE_LENGTH, 0.9f*OPT_EDGE_LENGTH);
        Vec2 v1 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v1, Float.MAX_VALUE);
        pos_order.add(v1);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), 0.9f*OPT_EDGE_LENGTH, OPT_EDGE_LENGTH);
        Vec2 v2 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v2, Float.MAX_VALUE);
        pos_order.add(v2);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), 0.9f*OPT_EDGE_LENGTH, 0.9f*OPT_EDGE_LENGTH);
        Vec2 v3 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v3, Float.MAX_VALUE);
        pos_order.add(v3);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), 0.7f*OPT_EDGE_LENGTH, 0.7f*OPT_EDGE_LENGTH);
        Vec2 v4 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v4, Float.MAX_VALUE);
        pos_order.add(v4);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), OPT_EDGE_LENGTH, 0.7f*OPT_EDGE_LENGTH);
        Vec2 v5 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v5, Float.MAX_VALUE);
        pos_order.add(v5);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), 0.7f*OPT_EDGE_LENGTH, OPT_EDGE_LENGTH);
        Vec2 v6 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v6, Float.MAX_VALUE);
        pos_order.add(v6);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), 0.6f*OPT_EDGE_LENGTH, 0.6f*OPT_EDGE_LENGTH);
        Vec2 v7 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v7, Float.MAX_VALUE);
        pos_order.add(v7);
        tmp = TriangleUtils.circleIntersection(edge_a, edge_b, edge_a.sub(edge_b).length(), 0.5f*OPT_EDGE_LENGTH, 0.5f*OPT_EDGE_LENGTH);
        Vec2 v8 = (edge.triangle.fuzzyContains(tmp[0])<edge.triangle.fuzzyContains(tmp[1])?tmp[0]:tmp[1]);
        ratings.put(v8, Float.MAX_VALUE);
        pos_order.add(v8);
    }

    /**
     * Has to be called at the beginning of every iteration of EdgeExpansion
     * @param controller The robot controller to provide neighborhood information
     */
    public void loop(DOTRobotController controller){
        if(controller.movementController.getLocalizationTriangle()==null){ System.err.println(controller.getID()+" could not perform extension point optimization due to missing locT"); return;}
        //Rate current position
        Vec2 pos = controller.movementController.locate(controller);
        float rating = rateExtensionPoint(controller, pos);
        if(best_save_pos == null || rating>best_save_pos_rating) { best_save_pos = pos; best_save_pos_rating = rating;}

        //Rate basic positions
        for(Map.Entry<Vec2, Float> e: ratings.entrySet()){
            if(pos.sub(e.getKey()).lengthSquared()<0.1f*0.1f) rating = rateExtensionPoint(controller, pos);
            else rating = rateExtensionPoint(controller, e.getKey());
            if(rating<e.getValue()) e.setValue(rating);
        }
    }

    /**
     * Returns the currently best position
     * @return best position
     */
    public Vec2 getBestPos(){
        float max_rating = 0;
        Vec2 max_pos = null;
        for(Vec2 p: pos_order) {
            if(max_pos == null || max_rating*1.01f<ratings.get(p)){ max_pos = p; max_rating = ratings.get(p);}
        }
        if(best_save_pos!=null && max_rating<best_save_pos_rating) return best_save_pos.clone();
        else return max_pos;
    }

    /**
     * Returns the currently best rating
     * @return Best rating
     */
    public float getRating(){
        float max_rating = 0;
        Vec2 max_pos = null;
        for(Map.Entry<Vec2, Float> e: ratings.entrySet()) {
            if(max_pos == null || max_rating<e.getValue()){
                max_rating = e.getValue();
                max_pos = e.getKey();
            }
        }
        if(best_save_pos!=null && max_rating<best_save_pos_rating) return best_save_pos_rating;
        else return max_rating;
    }

    /**
     * Rates an extension point. The rating is mainly based on the distances of points to edges of triangles.
     * The rating is between 0 (worst) and 0.9*OPT_EDGE_LENGTH (best)
     * Note: The extension point is not checked to provide a valid triangle.
     * @param controller The controller of the robot that also provides the neighborhood
     * @param extensionPoint The considered extension point
     * @return The rating for the triangle created by the frontier edge and this extension point
     */
    public float rateExtensionPoint(DOTRobotController controller, Vec2 extensionPoint){
        float min = 0.9f*OPT_EDGE_LENGTH; //Upper bound for rating. Too large distances aren't better so they should not get a better rating.

        //If edges too long -> return 0 (worst rating)
        if(edge_a.sub(extensionPoint).lengthSquared()>MAX_EDGE_LENGTH*MAX_EDGE_LENGTH){ return 0;}
        if(edge_b.sub(extensionPoint).lengthSquared()>MAX_EDGE_LENGTH*MAX_EDGE_LENGTH){ return 0;}

        Vec2 pos = frontierEdge.triangle.locate(controller); //Current position of robot
        //If the robot is close the the extension point, also use it's neighborhood information
        boolean use_distances = pos.sub(extensionPoint).lengthSquared()<TriangleMovementController.GOAL_TOLERANCE*TriangleMovementController.GOAL_TOLERANCE;
        HashMap<Integer, Vec2> static_nbr_pos = new HashMap<Integer, Vec2>(); //Filled in first for-loop, used in the second for-loop

        //Distances of points to edges of the triangle created by the extension point
        for(Neighbor nbr: controller.neighborManager.getNeighbors()) {
            if (nbr.publicVariables.isStatic) {
                Vec2 p = controller.movementController.getLocalizationTriangle().locateNbr(controller, nbr.ID);
                //TODO: Use own position to localize neighbors that cannot be localized yet
                if(p!=null) { //Corresponding robot could be localized
                    static_nbr_pos.put(nbr.ID, p);
                    //if(p.sub(extensionPoint).lengthSquared()>MAX_EDGE_LENGTH*MAX_EDGE_LENGTH) continue;
                    if(nbr.ID!=frontierEdge.firstRobot) {
                        float dist = MathUtils.sqrt(TriangleUtils.squaredDistanceToLineSegment(edge_a, extensionPoint, p));
                        if(dist<=MAX_RANGE) {
                            if (dist > MAX_EDGE_LENGTH) min *= 0.95f;
                            else min = Math.min(min, dist);
                        }
                    }
                    if(nbr.ID!=frontierEdge.secondRobot) {
                        float dist = MathUtils.sqrt(TriangleUtils.squaredDistanceToLineSegment(edge_b, extensionPoint, p));
                        if(dist<=MAX_RANGE) {
                            if (dist > MAX_EDGE_LENGTH) min *= 0.95f;
                            else min = Math.min(min, dist);
                        }
                    }
                } else if (use_distances){ //Corresponding robot could not be localized but we can use distance information
                    if(nbr.distance<=MAX_RANGE) {
                        if (nbr.distance > MAX_EDGE_LENGTH) min *= 0.95f; // penalize exterior long edges to prevent indirectly creating bad triangles
                        else min = Math.min(min, 0.8f * nbr.distance); //Point distances are usually larger than the corresponding edge distance
                    }
                }
            }
        }

        //Also rate the distance of the extension point to exterior edges
        for(Neighbor nbr: controller.neighborManager.getNeighbors()) {
            if(static_nbr_pos.containsKey(nbr.ID)){
                for(Integer n: nbr.publicVariables.frontier_pheromone.keySet()){
                    if(static_nbr_pos.containsKey(n)){
                        min = Math.min(min, MathUtils.sqrt(TriangleUtils.squaredDistanceToLineSegment(static_nbr_pos.get(nbr.ID), static_nbr_pos.get(n), extensionPoint)));
                    }
                }
            }
        }

        return min;
    }

    /**
     * Rates a single triangle without consideration of other points.
     * The rating is generally the distance of any point of the triangle to the corresponding opposite edge
     * @param a First point
     * @param b Second Point (ccw)
     * @param c Third Point
     * @return The rating
     */
    public static float rateTriangle(Vec2 a, Vec2 b, Vec2 c, float max_edge_length) {
        if (a == null || b == null || c == null) return 0;
        if (AngleUtils.normalizeToMinusPi_Pi(AngleUtils.getClockwiseRadian(c.sub(a), b.sub(a))) < 0) return 0; //Irregular triangle

        //Check if edges are ok
        float ab = a.sub(b).lengthSquared();
        float bc = b.sub(c).lengthSquared();
        float ca = c.sub(a).lengthSquared();
        float max_edge = MathUtils.sqrt(Math.max(ab, Math.max(bc, ca)));
        if (max_edge > max_edge_length) return 0;

        //Calculate and rate the distances
        float d1 = TriangleUtils.squaredDistanceToLineSegment(a, b, c);
        float d2 = TriangleUtils.squaredDistanceToLineSegment(a, c, b);
        float d3 = TriangleUtils.squaredDistanceToLineSegment(b, c, a);

        return MathUtils.sqrt(Math.min(d1, Math.min(d2, d3)));
    }
    public static float rateTriangle(Vec2 a, Vec2 b, Vec2 c){
        return rateTriangle(a,b,c, MAX_EDGE_LENGTH);
    }

    /**
     * Rates a single triangle without consideration of other points
     * The rating is generally the distance of any point of the triangle to the corresponding opposite edge
     * @param lt The triangle to rate
     * @return The rating
     */
    public static float rateTriangle(LocalizationTriangle lt, DOTRobotController controller) {
        for(Neighbor n: controller.neighborManager.getNeighbors()){
            if(n.publicVariables.isStatic){
                if(!lt.containsTriangleRobot(n.ID)){
                    Vec2 np = lt.locateNbr(controller, n.ID);
                    if(np!=null){
                        if(lt.contains(np)) return 0;
                    }
                }
            }
        }
        return rateTriangle(lt.pos_A, lt.pos_B, lt.pos_C);
    }

    /**
    public static Float rateCandidate(DOTRobotController controller, int candidate){

    }**/

}
