package projects.triangulation;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import platypus3000.utils.AngleUtils;
import platypus3000.utils.VectorUtils;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.NeighborManager;
import projects.triangulation.NeighborManager.PublicVariables;

import java.util.ArrayList;
import java.util.Random;


/**
 * This class provides a triangle relative coordination system and tools to work in it (localization, etc.).
 * It needs a static triangle of the triangulation (but only ids and distances measurements are used).
 * Used for robots with distance (and id) sensors only.
 * There is no orientation of the robot in the resulting coordinate system and the robot is not the origin.
 *
 * @author Dominik Krupke, January 2016
 */
public class LocalizationTriangle {
    //Ids of triangle robots
    public int robotA;
    public int robotB;
    public int robotC;
    /**
     * Checks if the given robot ID is the id of a robot of the three static triangle robots
     *
     * @param robotID
     * @return true if robot is part of triangle
     */
    public boolean containsTriangleRobot(int robotID) {
        return robotA == robotID || robotB == robotID || robotC == robotID;
    }

    //Pairwise distances between triangle robots
    float dist_ab;
    float dist_bc;
    float dist_ca;

    //Position of the three robots
    public Vec2 pos_A;
    public Vec2 pos_B;
    public Vec2 pos_C;
    public Vec2 getPosOfTriangleRobot(int robot) {
        if (robot == robotA) return pos_A.clone();
        if (robot == robotB) return pos_B.clone();
        if (robot == robotC) return pos_C.clone();
        assert false;
        return null;
    }




    //Update the distances in the triangle
    public boolean update(DOTRobotController controller){
        Neighbor neighborA = controller.neighborManager.getNeighbor(robotA);
        Neighbor neighborB = controller.neighborManager.getNeighbor(robotB);
        Neighbor neighborC = controller.neighborManager.getNeighbor(robotC);

        Float dist_ab = avg(new Float[]{(neighborA == null ? null : neighborA.publicVariables.distances.get(robotB)), (neighborB == null ? null : neighborB.publicVariables.distances.get(robotA))});
        Float dist_bc = avg(new Float[]{(neighborB == null ? null : neighborB.publicVariables.distances.get(robotC)), (neighborC == null ? null : neighborC.publicVariables.distances.get(robotB))});
        Float dist_ca = avg(new Float[]{(neighborC == null ? null : neighborC.publicVariables.distances.get(robotA)), (neighborA == null ? null : neighborA.publicVariables.distances.get(robotC))});

        if (dist_ab == null || dist_bc == null || dist_ca == null){
            System.err.println("Failed to update localization triangle "+toString());
            return false;
        }

        this.dist_ab = dist_ab;
        this.dist_bc = dist_bc;
        this.dist_ca = dist_ca;

        return true;
    }

    public LocalizationTriangle getAdjacentTriangle(DOTRobotController controller, Edge e) {
        assert e != null;
        Neighbor n1 = controller.neighborManager.getNeighbor(e.firstRobot);
        Neighbor n2 = controller.neighborManager.getNeighbor(e.secondRobot);

        ArrayList<PublicVariables.Triangle> triangles = n1.publicVariables.getIncidentTriangles(e.firstRobot, e.secondRobot);
        PublicVariables.Triangle triangle = null;
        for (PublicVariables.Triangle t : triangles) {
            if (t.contains(robotA) && t.contains(robotB) && t.contains(robotC)) continue; //Current triangle -> skip
            triangle = t;
        }
        if (triangle == null){
            System.err.println("Could not determine adjacent triangle");
            return null;
        }

        //Robot that is not in the edge
        int robotC = (containsTriangleRobot(triangle.robotC) ? (containsTriangleRobot(triangle.robotB) ? triangle.robotA : triangle.robotB) : triangle.robotC);
        Float ab = avg(new Float[]{n1.publicVariables.distances.get(e.secondRobot),n2.publicVariables.distances.get(e.firstRobot)});
        Float bc = n1.publicVariables.distances.get(robotC);
        Float ca = n2.publicVariables.distances.get(robotC);
        if(ab == null || bc ==null ||ca ==null) return null;
        return new LocalizationTriangle(e.secondRobot, e.firstRobot, robotC, ab, bc, ca);
    }

    /**
     * Transforms the point p in the coordinate system of neighbored_localization_triangle to this coordinate system.
     * Is not very efficient but simpler to use than returning a transformation matrix.
     * <p/>
     * If you want to transform a direction d, you can simply use
     * transfromFrom(..., d ,...)-transformFrom(..., (0,0), ...)
     *
     * @param neighbored_localization_triangle The triangle of the coordinate system of p
     * @param p                                The point to be transformed.
     * @return p in the coordinate system of this triangle
     */
    public Vec2 transformFrom(LocalizationTriangle neighbored_localization_triangle, Vec2 p, DOTRobotController controller) {

        Vec2 origin = locateNbr(controller, neighbored_localization_triangle.robotA);
        Vec2 xaxis = locateNbr(controller, neighbored_localization_triangle.robotB);
        if(origin==null || xaxis==null){
            System.err.println("Unable to transform");
            return null;
        }

        //The rotation difference between the two coordinate system
        float angle = AngleUtils.getRadian(xaxis.sub(origin));
        //Shift origin and rotate
        return VectorUtils.rotate(p, -angle).add(origin);
    }

    /**
     * Returns the center of the triangle
     *
     * @return Relative coordinates of the center of the triangle
     */
    public Vec2 getCenter() {
        return pos_B.add(pos_C).mul(1 / 3f); //posA = (0,0)
    }


    //************************************************************************************
    // GetTriangle and Constructor
    //************************************************************************************

    static private Float avg(Float[] d) {
        int n = 0;
        float r = 0;
        for (Float x : d) {
            if (x != null) {
                r += x;
                ++n;
            }
        }
        if (n == 0) return null;
        return r / n;
    }

    //Returns all triangles that the robot can view (at least two robots of it have to be in range and it has to be in the public variables)
    public static ArrayList<LocalizationTriangle> findTriangles(DOTRobotController controller) {
        ArrayList<LocalizationTriangle> ret = new ArrayList<LocalizationTriangle>(); //The set of all found triangles

        //Iterate over all triangles in the neighborhood
        for (Neighbor nbr : controller.neighborManager.getNeighbors()) {
            for (PublicVariables.Triangle t : nbr.publicVariables.triangles) {
                //Get triangle's robots
                Neighbor tA = controller.neighborManager.getNeighbor(t.robotA);
                Neighbor tB = controller.neighborManager.getNeighbor(t.robotB);
                Neighbor tC = controller.neighborManager.getNeighbor(t.robotC);

                //Two neighbored robots are needed for a localization
                if((tA!=null?1:0)+(tB!=null?1:0)+(tC!=null?1:0)<2) continue;

                //Distances between the triangle robots
                Float dist_ab = avg(new Float[]{(tA == null ? null : tA.publicVariables.distances.get(t.robotB)), (tB == null ? null : tB.publicVariables.distances.get(t.robotA))});
                Float dist_bc = avg(new Float[]{(tB == null ? null : tB.publicVariables.distances.get(t.robotC)), (tC == null ? null : tC.publicVariables.distances.get(t.robotB))});
                Float dist_ca = avg(new Float[]{(tC == null ? null : tC.publicVariables.distances.get(t.robotA)), (tA == null ? null : tA.publicVariables.distances.get(t.robotC))});
                if (dist_ab == null || dist_bc == null || dist_ca == null) continue;

                //Create LocalizationTriangle and add them to the set if they are not yet in
                LocalizationTriangle lt = new LocalizationTriangle(t.robotA, t.robotB, t.robotC, dist_ab, dist_bc, dist_ca);
                if (!ret.contains(lt)) ret.add(lt);
            }
        }
        return ret;
    }

    //Returns all triangles the robot is fuzzy contained to the given value.
    public static ArrayList<LocalizationTriangle> findTriangles(DOTRobotController controller, float min_fuzzy_contain) {
        ArrayList<LocalizationTriangle> ret = new ArrayList<LocalizationTriangle>();
        for (LocalizationTriangle l : findTriangles(controller)) {
            if (l.fuzzyContains(controller) >= min_fuzzy_contain) ret.add(l);
        }
        return ret;
    }

    /**
     * Returns the closest triangle. Use contains or fuzzyContains to check for inclusion.
     *
     * @param controller It's controller
     * @return Closest triangle
     */
    public static LocalizationTriangle getClosestTriangle(DOTRobotController controller) {
        LocalizationTriangle bestTriangle = null;
        float bestValue = 0;
        for (LocalizationTriangle lt : findTriangles(controller)) {

            float fuzzyValue = lt.fuzzyContains(lt.locate(controller));

            if (bestTriangle == null || fuzzyValue > bestValue) {
                bestTriangle = lt;
                bestValue = fuzzyValue;
            }
        }
        return bestTriangle;
    }

    public static LocalizationTriangle create(int robotA, int robotB, int robotC, DOTRobotController controller){
        Neighbor nA = controller.neighborManager.getNeighbor(robotA);
        Neighbor nB = controller.neighborManager.getNeighbor(robotB);
        Neighbor nC = controller.neighborManager.getNeighbor(robotC);

        Float dist_ab = avg(new Float[]{(nA == null ? null : nA.publicVariables.distances.get(robotB)), (nB == null ? null : nB.publicVariables.distances.get(robotA))});
        Float dist_bc = avg(new Float[]{(nB == null ? null : nB.publicVariables.distances.get(robotC)), (nC == null ? null : nC.publicVariables.distances.get(robotB))});
        Float dist_ca = avg(new Float[]{(nC == null ? null : nC.publicVariables.distances.get(robotA)), (nA == null ? null : nA.publicVariables.distances.get(robotC))});

        if (dist_ab == null || dist_bc == null || dist_ca == null){
            return null;
        }
        return new LocalizationTriangle(robotA, robotB, robotC, dist_ab, dist_bc, dist_ca);
    }



    // Sets the positions of triangle robots by calculating them of the distances
    private void calculatePositionsFromDistances(){
        pos_A = new Vec2(0, 0);
        pos_B = new Vec2(dist_ab, 0);
        float alpha = (float) Math.acos((dist_ca * dist_ca + dist_ab * dist_ab - dist_bc * dist_bc) / (2 * dist_ca * dist_ab));
        pos_C = new Vec2(MathUtils.cos(alpha) * dist_ca, MathUtils.sin(alpha) * dist_ca);
    }

    /**
     * Constructor. Builds the triangle relative coordinate system using only ids and distances.
     * The robots have to be in counterclockwise order (which robot is first does not matter).
     * The ids are needed to choose a definite origin
     *
     * @param robotA  First (counterclockwise) robot's id
     * @param robotB  Second (counterclockwise) robot's id
     * @param robotC  Third (counterclockwise) robot's id
     * @param dist_ab Distance between first and second robot
     * @param dist_bc Distance between second and third robot
     * @param dist_ca Distance between third and first robot
     */
    public LocalizationTriangle(int robotA, int robotB, int robotC, float dist_ab, float dist_bc, float dist_ca) {
        while (robotB < robotA || robotC < robotA) { //rotate until robot a is minimal
            int tmp = robotA;
            robotA = robotB;
            robotB = robotC;
            robotC = tmp;
            float tmp2 = dist_ab;
            dist_ab = dist_bc;
            dist_bc = dist_ca;
            dist_ca = tmp2;
        }

        this.robotA = robotA;
        this.robotB = robotB;
        this.robotC = robotC;
        this.dist_ab = dist_ab;
        this.dist_bc = dist_bc;
        this.dist_ca = dist_ca;

        calculatePositionsFromDistances();
    }

    public PublicVariables.Triangle triangle(){
        return new PublicVariables.Triangle(robotA, robotB, robotC);
    }

    //************************************************************************************
    // Locate
    //************************************************************************************

    /**
     * Returns the position in the triangle relative coordinate system calculated from the distances to the triangle
     * robots (automatically taken from nbrs).
     *
     * @param robot The robot to be localized
     * @return A position estimation in the triangle relative coordinate system
     */
    public Vec2 locate(DOTRobotController robot) {
        NeighborManager nbrs = robot.neighborManager;
        Float d1 = (nbrs.contains(robotA)?nbrs.getNeighbor(robotA).distance:null);
        Float d2 = (nbrs.contains(robotB)?nbrs.getNeighbor(robotB).distance:null);
        Float d3 = (nbrs.contains(robotC)?nbrs.getNeighbor(robotC).distance:null);
        return beacon3Localization(pos_A, pos_B, pos_C, d1, d2, d3);
    }

    public Vec2 locate(DOTRobotController robot, Vec2 prev_pos){
        if(prev_pos==null) return locate(robot);
        Vec2[] positions = ambiguousLocalize(robot);
        if(positions.length==1) return positions[0];
        if(positions.length==0) return null;
        return (positions[0].sub(prev_pos).lengthSquared()<positions[1].sub(prev_pos).lengthSquared()?positions[0]:positions[1]);
    }

    /**
     * Localizes a neighbor
     *
     * @param nbr
     * @return
     */
    public Vec2 locateNbr(DOTRobotController controller, int nbr) {
        if (nbr == robotA) return pos_A;
        if (nbr == robotB) return pos_B;
        if (nbr == robotC) return pos_C;

        Neighbor n = controller.neighborManager.getNeighbor(nbr);
        if(n == null) { System.err.println("LocalizationTriangle.locateNbr: No NeighborState of "+nbr+", returning null");return null;}
        Float d1 = n.publicVariables.distances.get(robotA);
        Float d2 = n.publicVariables.distances.get(robotB);
        Float d3 = n.publicVariables.distances.get(robotC);
        Vec2 nbrPos = beacon3Localization(pos_A, pos_B, pos_C, d1, d2, d3);

        if(nbrPos == null && !containsTriangleRobot(controller.getID())){
            Vec2 own_pos = locate(controller);
            if(own_pos!=null && fuzzyContains(own_pos)>0.2f){
                Edge e = getClosestEdge(controller);
                LocalizationTriangle lt = create(e.secondRobot, e.firstRobot, controller.getID(), controller);
                if(lt!=null && ExtensionPointOptimization.rateTriangle(lt, controller)>0.3f){
                    nbrPos = lt.locateNbr(controller, nbr);
                    if(nbrPos!=null) return transformFrom(lt, nbrPos, controller);
                }
            }
        }
        return nbrPos;
    }

    /**
     * Makes a localization using three beacons with known positions (in an arbitrary but fixed coordinate system) and
     * at least distances to two of the beacons. The distances can be inaccurate.
     *
     * @param b1 Coordinates of beacon 1
     * @param b2 Coordinates of beacon 2
     * @param b3 Coordinates of beacon 3
     * @param d1 Distance to beacon 1 or -1 if not available
     * @param d2 Distance to beacon 2 or -1 if not available
     * @param d3 Distance to beacon 3 or -1 if not available
     * @return The average of the most probable locations deduced from the input.
     */
    private Vec2 beacon3Localization(Vec2 b1, Vec2 b2, Vec2 b3, Float d1, Float d2, Float d3) {
        assert (d1==null|| d1>0) && (d2==null ||d2>0) && (d3==null ||d3>0);
        assert b1!=null && b2!=null && b3!=null;
        if((d1!=null?1:0)+(d2!=null?1:0)+(d3!=null?1:0)<2) return null; //Not enough information

        Vec2 p1_1 = (d1 != null && d2 != null ? TriangleUtils.circleIntersection(b1, b2, b1.sub(b2).length(), d1, d2)[0] : null);
        Vec2 p1_2 = (d1 != null && d2 != null ? TriangleUtils.circleIntersection(b1, b2, b1.sub(b2).length(), d1, d2)[1] : null);
        Vec2 p2_1 = (d3 != null && d2 != null ? TriangleUtils.circleIntersection(b3, b2, b3.sub(b2).length(), d3, d2)[0] : null);
        Vec2 p2_2 = (d3 != null && d2 != null ? TriangleUtils.circleIntersection(b3, b2, b3.sub(b2).length(), d3, d2)[1] : null);
        Vec2 p3_1 = (d1 != null && d3 != null ? TriangleUtils.circleIntersection(b1, b3, b1.sub(b3).length(), d1, d3)[0] : null);
        Vec2 p3_2 = (d1 != null && d3 != null ? TriangleUtils.circleIntersection(b1, b3, b1.sub(b3).length(), d1, d3)[1] : null);

        Vec2 bestPos = null;
        float diff = 0;
        for (int selection = 0; selection < 8; selection++) {
            Vec2 p1 = ((selection & 0x1) > 0 ? p1_1 : p1_2);
            Vec2 p2 = ((selection & 0x2) > 0 ? p2_1 : p2_2);
            Vec2 p3 = ((selection & 0x4) > 0 ? p3_1 : p3_2);
            Vec2 avg = new Vec2();
            int count = 0;
            if (p1 != null) {
                avg.addLocal(p1);
                ++count;
            }
            if (p2 != null) {
                avg.addLocal(p2);
                ++count;
            }
            if (p3 != null) {
                avg.addLocal(p3);
                ++count;
            }
            avg.mulLocal(1f / (count));
            float max_diff = 0;
            if (p1 != null && p1.sub(avg).lengthSquared() > max_diff) max_diff = p1.sub(avg).lengthSquared();
            if (p2 != null && p2.sub(avg).lengthSquared() > max_diff) max_diff = p2.sub(avg).lengthSquared();
            if (p3 != null && p3.sub(avg).lengthSquared() > max_diff) max_diff = p3.sub(avg).lengthSquared();

            //If only two selected, choose furthest to missing. Could be better implemented in own if
            if (d1 == null) max_diff = -b1.sub(avg).lengthSquared();
            if (d2 == null) max_diff = -b2.sub(avg).lengthSquared();
            if (d3 == null) max_diff = -b3.sub(avg).lengthSquared();

            if (bestPos == null || max_diff < diff) {
                bestPos = avg;
                diff = max_diff;
            }
        }
        //System.out.println("\t-> "+bestPos);
        return bestPos;
    }

    public Vec2[] ambiguousLocalize(DOTRobotController controller){
        NeighborManager nbrs = controller.neighborManager;
        Float d1 = (nbrs.contains(robotA)?nbrs.getNeighbor(robotA).distance:null);
        Float d2 = (nbrs.contains(robotB)?nbrs.getNeighbor(robotB).distance:null);
        Float d3 = (nbrs.contains(robotC)?nbrs.getNeighbor(robotC).distance:null);

        int number_of_known_distances = (d1!=null?1:0)+(d2!=null?1:0)+(d3!=null?1:0);
        if(number_of_known_distances<2) return new Vec2[0]; //Not enough information

        if(number_of_known_distances==2){
            Vec2 c1, c2;
            float dist_c1, dist_c2;
            if(d1==null){ c1=pos_B; dist_c1=d2; }
            else { c1 = pos_A; dist_c1 = d1; }
            if(d3==null){ c2 = pos_B; dist_c2=d2;}
            else { c2 = pos_C; dist_c2=d3;}
            return TriangleUtils.circleIntersection(c1, c2, c1.sub(c2).length(), dist_c1, dist_c2);
        } else {
            Vec2 p1_1 = TriangleUtils.circleIntersection(pos_A, pos_B, dist_ab, d1, d2)[0];
            Vec2 p1_2 = TriangleUtils.circleIntersection(pos_A, pos_B, dist_ab, d1, d2)[1];
            Vec2 p2_1 = TriangleUtils.circleIntersection(pos_B, pos_C, dist_bc, d2, d3)[0];
            Vec2 p2_2 = TriangleUtils.circleIntersection(pos_B, pos_C, dist_bc, d2, d3)[1];
            Vec2 p3_1 = TriangleUtils.circleIntersection(pos_C, pos_A, dist_ca, d3, d1)[0];
            Vec2 p3_2 = TriangleUtils.circleIntersection(pos_C, pos_A, dist_ca, d3, d1)[1];

            Vec2 bestPos = null;
            float diff = 0;
            for (int selection = 0; selection < 8; selection++) {
                Vec2 p1 = ((selection & 0x1) > 0 ? p1_1 : p1_2);
                Vec2 p2 = ((selection & 0x2) > 0 ? p2_1 : p2_2);
                Vec2 p3 = ((selection & 0x4) > 0 ? p3_1 : p3_2);
                Vec2 avg = new Vec2();
                int count = 0;
                if (p1 != null) {
                    avg.addLocal(p1);
                    ++count;
                }
                if (p2 != null) {
                    avg.addLocal(p2);
                    ++count;
                }
                if (p3 != null) {
                    avg.addLocal(p3);
                    ++count;
                }
                avg.mulLocal(1f / (count));
                float max_diff = 0;
                if (p1 != null && p1.sub(avg).lengthSquared() > max_diff) max_diff = p1.sub(avg).lengthSquared();
                if (p2 != null && p2.sub(avg).lengthSquared() > max_diff) max_diff = p2.sub(avg).lengthSquared();
                if (p3 != null && p3.sub(avg).lengthSquared() > max_diff) max_diff = p3.sub(avg).lengthSquared();

                if (bestPos == null || max_diff < diff) {
                    bestPos = avg;
                    diff = max_diff;
                }
            }
            //System.out.println("\t-> "+bestPos);
            return new Vec2[]{bestPos};
        }
    }


    //************************************************************************************
    // Contains
    //************************************************************************************

    /**
     * A rated contains (@see boolean contains(Vec2 p) ) that returns the positive distance to the triangle boundary
     * if it is in the triangle and the negative if not.
     * The closer the value is to 0 the more unreliable the decision for encompassion is.
     *
     * @param p The point to be checked (in triangle relative coordinate system)
     * @return Distance to boundary of triangle. >0 if in triangle, <0 if not
     */
    public float fuzzyContains(Vec2 p) {
        assert p!=null;
        float minDist = MathUtils.min(TriangleUtils.squaredDistanceToLineSegment(pos_A, pos_B, p), TriangleUtils.squaredDistanceToLineSegment(pos_B, pos_C, p));
        minDist = MathUtils.sqrt(MathUtils.min(minDist, TriangleUtils.squaredDistanceToLineSegment(pos_C, pos_A, p)));
        return (contains(p) ? minDist : -minDist);
    }

    public float fuzzyContains(DOTRobotController robot) {
        Vec2 p = locate(robot);
        if(p==null){
            System.err.println(robot.getID()+": Failed to check containment of robot "+robot.getID()+" in LT "+toString());
            return Float.MIN_VALUE;
        }
        return fuzzyContains(locate(robot));
    }

    /**
     * Returns true if point p is in the triangle. Can be inaccurate close to the boundary!
     * Adapted from http://totologic.blogspot.fr/2014/01/accurate-point-in-triangle-test.html
     *
     * @param p A point in the triangle relative coordinate system
     * @return true if in triangle, false if not
     */
    boolean contains(Vec2 p) {
        float a = ((pos_B.y - pos_C.y) * (p.x - pos_C.x) + (pos_C.x - pos_B.x) * (p.y - pos_C.y)) / ((pos_B.y - pos_C.y) * (pos_A.x - pos_C.x) + (pos_C.x - pos_B.x) * (pos_A.y - pos_C.y));
        float b = ((pos_C.y - pos_A.y) * (p.x - pos_C.x) + (pos_A.x - pos_C.x) * (p.y - pos_C.y)) / ((pos_B.y - pos_C.y) * (pos_A.x - pos_C.x) + (pos_C.x - pos_B.x) * (pos_A.y - pos_C.y));
        float c = 1 - a - b;
        return a > 0 && b > 0 && c > 0;
    }

    public boolean contains(DOTRobotController robot) {
        return fuzzyContains(robot) >= 0;
    }

    //************************************************************************************
    // Java - Functions
    //************************************************************************************

    /**
     * Returns true if the robot ids of both triangles coincide. The distances are neglected.
     *
     * @param o The triangle to compare
     * @return true if equals, false else
     */
    public boolean equals(Object o) {
        if (o instanceof LocalizationTriangle) {
            LocalizationTriangle lt = (LocalizationTriangle) o;
            return lt.robotA == robotA && lt.robotB == robotB && lt.robotC == robotC;
        }
        return false;
    }

    public LocalizationTriangle clone() {
        LocalizationTriangle cloned = new LocalizationTriangle(robotA, robotB, robotC, dist_ab, dist_bc, dist_ca);
        return cloned;
    }

    public String toString() {
        return "LocalizationTriangle(" + robotA + ", " + robotB + ", " + robotC + ")";
    }


    //************************************************************************************
    // Edges
    //************************************************************************************

    public Edge getEdgeAB() {
        return new Edge(robotA, robotB, this);
    }

    public Edge getEdgeBC() {
        return new Edge(robotB, robotC, this);
    }

    public Edge getEdgeCA() {
        return new Edge(robotC, robotA, this);
    }

    /**
     * Returns the closest edge to a robot of the triangle
     *
     * @param robot The corresponding robot
     * @return Edge closest to assumed position of robot
     */
    public Edge getClosestEdge(DOTRobotController robot) {
        Vec2 p = locate(robot);
        float dist_ab = getEdgeAB().getCenter().sub(p).lengthSquared();
        float dist_bc = getEdgeBC().getCenter().sub(p).lengthSquared();
        float dist_ca = getEdgeCA().getCenter().sub(p).lengthSquared();
        float dist_min = Math.min(dist_ab, Math.min(dist_bc, dist_ca));
        if (dist_ab == dist_min) return getEdgeAB();
        if (dist_bc == dist_min) return getEdgeBC();
        if (dist_ca == dist_min) return getEdgeCA();
        assert false;
        return null;
    }

    public class Edge {
        public int firstRobot;
        public int secondRobot;
        public LocalizationTriangle triangle;

        /**
         * Returns the optimal point to place a robot on the other side.
         * Can be used as goal for expanding an edge
         *
         * @return Optimal point for a robot to build a triangle on the other side of the edge
         */
        public Vec2 getExtensionPoint() {
            Vec2 c = getCenter();
            Vec2 d = triangle.getPosOfTriangleRobot(secondRobot).sub(c);
            d.normalize(); d.mulLocal(0.8f);
            return c.add(VectorUtils.rotate(d,-MathUtils.HALF_PI));
        }

        /**
         * Returns a point that can be used as goal for leaving the triangle on this edge.
         *
         * @return A point that lies outside the triangle (middle of this triangle mirrored on ca-edge)
         */
        public Vec2 getExitPoint() {
            Vec2 c = getCenter();
            Vec2 d = triangle.getPosOfTriangleRobot(secondRobot).sub(c);
            d.normalize(); d.mulLocal(0.4f);
            return c.add(VectorUtils.rotate(d,-MathUtils.HALF_PI));
        }

        public Vec2 getCenter() {
            return triangle.getPosOfTriangleRobot(firstRobot).add(triangle.getPosOfTriangleRobot(secondRobot)).mul(0.5f);
        }

        private Edge(int first, int second, LocalizationTriangle t) {
            this.firstRobot = first;
            this.secondRobot = second;
            this.triangle = t;
        }

        public boolean equals(Object o) {
            if (o instanceof Edge) {
                Edge e = (Edge) o;
                return this.triangle.equals(triangle) && e.firstRobot == firstRobot && e.secondRobot == secondRobot;
            }
            return false;
        }
    }

    public static Random rand = new Random();
    public static float by = 0.01f;
    public static float perturbate(float x){
        return rand.nextFloat()*by*(rand.nextBoolean()?1:-1)*x+x;
    }

    public static void main(String[] args){//TODO remove
        LocalizationTriangle lt = new LocalizationTriangle(1,2,3, 1,1,1);
        float center_dist = (float)(Math.sqrt(3)/4);
        ArrayList<Float> x = new ArrayList<Float>();
        ArrayList<Float> y = new ArrayList<Float>();
        System.out.println(lt.pos_A+" "+lt.pos_B+" "+lt.pos_C);
        System.exit(0);
        Random rand = new Random();
        for(int i=0; i<100; i++) System.out.println(perturbate(1));
        int N = 10000;
        for(int i=0; i<N; i++) {
            Vec2 p = lt.beacon3Localization(lt.pos_A, lt.pos_B, lt.pos_C, perturbate(center_dist), perturbate(center_dist), perturbate(center_dist));
            x.add(p.x);
            y.add(p.y);
        }
        System.out.println();
        System.out.print("xdat = [");
        for(int i=0; i<N; i++){
            if(i!=0) System.out.print(", ");
            System.out.print(x.get(i));
        }
        System.out.print("]\n");
        System.out.println();
        System.out.print("ydat = [");
        for(int i=0; i<N; i++){
            if(i!=0) System.out.print(", ");
            System.out.print(y.get(i));
        }
        System.out.print("]\n");
    }
}
