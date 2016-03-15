package projects.triangulation.NeighborManager;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;
import projects.triangulation.LocalizationTriangle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Stack;

/**
 * This class provides the public variables of the robots.
 * There is no discrimination between the robot states as in the simulator we have unlimited bandwidth between the robots.
 *
 * @author Dominik Krupke
 * Last modified: 12. Feb. 2016
 */
public class PublicVariables {
    //All robots
    public final int ID;
    public boolean isStatic = false; //Descriminates between free and static robots
    public HashMap<Integer, Float> distances = new HashMap<Integer, Float>();//2-hop neighborhood

    //Static Robots
    public ArrayList<Triangle> triangles = new ArrayList<Triangle>(); //incident triangles
    public HashMap<Integer, Integer> frontier_pheromone = new HashMap<Integer, Integer>();
    public HashMap<Integer, Integer> evasion_pheromone = new HashMap<Integer, Integer>();
    public ArrayList<Integer> wall_edges = new ArrayList<Integer>(4);

    //Free Robots
    public Triangle in_triangle;
    public Float expansion_value = null;

    //Partition
    public Integer partition_root = null;
    public int partition_hops= Integer.MAX_VALUE;
    public boolean partition_stable = false;
    public boolean partition_frozen = false;
    public Integer partition_parent = null;
    public boolean partition_margin = false;
    public int partition_supersize = 0;
    public int partition_subsize = 0;
    public int frontier_distance = Integer.MAX_VALUE;

    //Debugging
    public String toString(){
        StringBuilder returnString = new StringBuilder();
        returnString.append("ID=").append(ID).append('\n');
        returnString.append("isStatic=").append(isStatic).append('\n');
        returnString.append("distances=");
        for(Map.Entry<Integer, Float> e: distances.entrySet()){
            returnString.append("(").append(e.getKey()).append(", ").append(e.getValue()).append(") ");
        }
        returnString.append('\n');
        returnString.append("triangles=");
        for(Triangle t: triangles){
            returnString.append(t).append(", ");
        }
        returnString.append('\n');
        returnString.append("frontier_pheromone=");
        for(Map.Entry<Integer, Integer> e: frontier_pheromone.entrySet()){
            returnString.append("(").append(e.getKey()).append(", ").append(e.getValue()).append("), ");
        }
        returnString.append('\n');
        returnString.append("evasion_pheromone=");
        for(Map.Entry<Integer, Integer> e: evasion_pheromone.entrySet()){
            returnString.append("(").append(e.getKey()).append(", ").append(e.getValue()).append("), ");
        }
        returnString.append('\n');
        returnString.append("wall_edges=");
        for(Integer i: wall_edges){
            returnString.append(i).append(", ");
        }
        returnString.append('\n');

        returnString.append("in_triangle=").append(in_triangle).append('\n');
        returnString.append("expansion_value=").append(expansion_value).append('\n');

        returnString.append("partition_root="+partition_root+"\n");
        returnString.append("partition_hops="+partition_hops+"\n");
        returnString.append("partition_stable="+partition_stable+"\n");
        returnString.append("partition_frozen="+partition_frozen+"\n");
        returnString.append("partition_parent="+partition_parent+"\n");
        returnString.append("partition_margin="+partition_margin+"\n");
        returnString.append("partition_supersize="+partition_supersize+"\n");
        returnString.append("partition_subsize="+partition_subsize+"\n");
        returnString.append("frontier_distance="+frontier_distance+"\n");

        //TODO
        return returnString.toString();
    }

    public PublicVariables(int ID){
        this.ID  = ID;
    }

    public boolean isFrontierOrWall(){
        //Find edges that are only in one triangle and not marked as wall edges -> frontier edges -> dist 0
        HashMap<Integer, Integer> in_n_triangles = new HashMap<Integer, Integer>();
        for(PublicVariables.Triangle t: this.triangles){
            if(in_n_triangles.containsKey(t.robotA)) in_n_triangles.put(t.robotA, in_n_triangles.get(t.robotA)+1);
            else in_n_triangles.put(t.robotA, 1);
            if(in_n_triangles.containsKey(t.robotB)) in_n_triangles.put(t.robotB, in_n_triangles.get(t.robotB)+1);
            else in_n_triangles.put(t.robotB, 1);
            if(in_n_triangles.containsKey(t.robotC)) in_n_triangles.put(t.robotC, in_n_triangles.get(t.robotC)+1);
            else in_n_triangles.put(t.robotC, 1);
        }
        in_n_triangles.remove(ID); //remove the own count as it doesn't actually represent an edge
        for(Map.Entry<Integer, Integer> e: in_n_triangles.entrySet()){
            if(e.getValue()==1) return true;
        }
        return false;
    }

    public PublicVariables createMessageClone(){
        PublicVariables cloned = new PublicVariables(ID);
        cloned.isStatic = this.isStatic;
        cloned.distances.putAll(this.distances);
        for (Triangle t : triangles) {
            cloned.triangles.add(t.clone());
        }
        for(Integer i: wall_edges) {
            cloned.wall_edges.add(i);
        }
        cloned.in_triangle = in_triangle;
        cloned.evasion_pheromone = new HashMap<Integer, Integer>(evasion_pheromone);
        cloned.frontier_pheromone = new HashMap<Integer, Integer>(frontier_pheromone);
        cloned.expansion_value = this.expansion_value;

        //Partition
        cloned.partition_hops = partition_hops;
        cloned.partition_root = partition_root;
        cloned.partition_stable = partition_stable;
        cloned.partition_frozen = partition_frozen;
        cloned.partition_parent = partition_parent;
        cloned.partition_margin = partition_margin;
        cloned.partition_subsize = partition_subsize;
        cloned.partition_supersize = partition_supersize;
        cloned.frontier_distance = frontier_distance;

        return cloned;
    }

    public int minFrontierPheromone(DOTRobotController controller){
        int min = Integer.MAX_VALUE;
        for(Map.Entry<Integer,Integer> e: frontier_pheromone.entrySet()){
            if(e.getValue()<min) min = e.getValue();
        }
        for(Neighbor nbr: controller.neighborManager.getNeighbors()){
            if(nbr.publicVariables.isStatic){
                if(nbr.publicVariables.frontier_pheromone.containsKey(controller.getID())){
                    int val = nbr.publicVariables.frontier_pheromone.get(controller.getID());
                    if(val<min) min = val;
                }
            }
        }
        return min;
    }

    public void deleteStaticRobotVariables(){
        triangles.clear();
        frontier_pheromone.clear();
        wall_edges.clear();
        evasion_pheromone.clear();
        deletePartitionVariables();
    }

    public void deletePartitionVariables(){
        partition_hops = Integer.MAX_VALUE;
        partition_root = null;
        partition_frozen = false;
        partition_stable = false;
        partition_parent = null;
        frontier_distance = Integer.MAX_VALUE;
    }

    public ArrayList<Triangle> getIncidentTriangles(int robotA, int robotB){
        ArrayList<Triangle> ret = new ArrayList<Triangle>(2);
        for(Triangle t: triangles){
            if(t.contains(robotA) && t.contains(robotB)) ret.add(t);
        }
        return ret;
    }

    public void addTriangle(Triangle t){
        if(!triangles.contains(t))  triangles.add(t);
    }

    public boolean manages_a_frontier_edge() {
        for(Map.Entry<Integer, Integer> e: frontier_pheromone.entrySet()){
            if(e.getValue()==0) return true;
        }
        return false;
    }

    public void reset() {
        isStatic = false;
        distances.clear();

        triangles.clear();
        frontier_pheromone.clear();
        evasion_pheromone.clear();
        wall_edges.clear();

        //Free Robots
        in_triangle=null;
        expansion_value = null;

        //Partition
        partition_root = null;
        partition_hops = Integer.MAX_VALUE;
        partition_stable = false;
        partition_frozen = false;
        partition_parent = null;
        partition_margin = false;
    }

    public boolean isFrontier(DOTRobotController controller) {
        if(!isStatic) return false;
        return minFrontierPheromone(controller)==0;
    }

    public static class Triangle {
        public int robotA;
        public int robotB;
        public int robotC;

        public Triangle(int A, int B, int C) {
            this.robotA = A;
            this.robotB = B;
            this.robotC = C;
        }

        public Triangle(LocalizationTriangle localizationTriangle) {
            this(localizationTriangle.robotA, localizationTriangle.robotB, localizationTriangle.robotC);
        }

        public boolean contains(int robot) {
            return robotA == robot || robotB == robot || robotC == robot;
        }

        @Override
        public boolean equals(Object o) {
            if(o instanceof Triangle){
                Triangle t = (Triangle)o;
                return contains(t.robotA) && contains(t.robotB) && contains(t.robotC);
            } else {
                return false;
            }
        }

        @Override
        public Triangle clone() {
            return new Triangle(robotA, robotB, robotC);
        }

        @Override
        public String toString(){
            return "("+robotA+", "+robotB+", "+robotC+")";
        }
    }

    public int getFrontierPheromone(DOTRobotController controller, int robotA, int robotB){
        int correspondRobotID = Math.min(robotA, robotB);
        if(correspondRobotID==controller.getID()){
            if(!this.frontier_pheromone.containsKey(Math.max(robotA, robotB))) return Integer.MAX_VALUE;
            return this.frontier_pheromone.get(Math.max(robotA, robotB));
        }
        Neighbor correspondingNeighbor = controller.neighborManager.getNeighbor(correspondRobotID);
        if(correspondingNeighbor==null){
            //System.err.println(controller.getID()+": PublicVariables.getEvasionPheromone didn't find neighbor "+correspondRobotID +", returning Integer.MAX_VALUE");
            return  Integer.MAX_VALUE;
        }
        if(!correspondingNeighbor.publicVariables.frontier_pheromone.containsKey(Math.max(robotA, robotB))) return  Integer.MAX_VALUE;
        return correspondingNeighbor.publicVariables.frontier_pheromone.get(Math.max(robotA, robotB));
    }

    public void update_frontier_pherome(RobotInterface robot, DOTRobotController controller, ArrayList<Integer> blacklist) {
        if(!this.isStatic){ frontier_pheromone.clear(); return;}
        HashMap<Integer, Integer> new_hop_distances = new HashMap<Integer, Integer>(this.frontier_pheromone.size());

        //Remove invalid triangles
        Stack<Triangle> toRemove = new Stack<PublicVariables.Triangle>();
        for(PublicVariables.Triangle t: this.triangles){
            if(t.robotA!=controller.getID() && !controller.neighborManager.contains(t.robotA)) toRemove.push(t);
            else if(t.robotB!=controller.getID() && !controller.neighborManager.contains(t.robotB)) toRemove.push(t);
            else if(t.robotC!=controller.getID() && !controller.neighborManager.contains(t.robotC)) toRemove.push(t);
        }
        for(PublicVariables.Triangle t: toRemove){
            this.triangles.remove(t);
        }

        //Find edges that are only in one triangle and not marked as wall edges -> frontier edges -> dist 0
        HashMap<Integer, Integer> in_n_triangles = new HashMap<Integer, Integer>();
        for(PublicVariables.Triangle t: this.triangles){
            if(in_n_triangles.containsKey(t.robotA)) in_n_triangles.put(t.robotA, in_n_triangles.get(t.robotA)+1);
            else in_n_triangles.put(t.robotA, 1);
            if(in_n_triangles.containsKey(t.robotB)) in_n_triangles.put(t.robotB, in_n_triangles.get(t.robotB)+1);
            else in_n_triangles.put(t.robotB, 1);
            if(in_n_triangles.containsKey(t.robotC)) in_n_triangles.put(t.robotC, in_n_triangles.get(t.robotC)+1);
            else in_n_triangles.put(t.robotC, 1);
        }
        in_n_triangles.remove(controller.getID()); //remove the own count as it doesn't actually represent an edge
        for(Map.Entry<Integer, Integer> e: in_n_triangles.entrySet()){
            if(e.getValue()==1 && !this.wall_edges.contains(e.getKey()) && e.getKey()>controller.getID() && (!partition_frozen && !blacklist.contains(e.getKey()))) new_hop_distances.put(e.getKey(), 0);
            if(e.getValue()==2 ){
                if(controller.expansion_trials.containsKey(e.getKey())) controller.expansion_trials.remove(e.getKey());
                if(this.wall_edges.contains(e.getKey())) this.wall_edges.remove(e.getKey());//If an wall edge has two adjacent triangles it is obviously no wall edge
            }
            if(e.getValue()>2){
                System.err.println(controller.getID()+": An edge "+e.getKey()+" is in more than 2 triangles -> inconsistent"); controller.delete(robot, "Has an edge in more than 2 triangles");}
        }

        //TODO: Prohibit the usage of edges with evasion>0
        //Calculate inductive pheromone value
        for(PublicVariables.Triangle t: this.triangles){
            int ab = getFrontierPheromone(controller, t.robotA, t.robotB);
            int bc = getFrontierPheromone(controller, t.robotB, t.robotC);
            int ca = getFrontierPheromone(controller, t.robotC, t.robotA);
            int min = Math.min(ab,Math.min(bc,ca));
            if(min!=Integer.MAX_VALUE) min+=1;
            if(controller.getID()<t.robotA && (!new_hop_distances.containsKey(t.robotA) || new_hop_distances.get(t.robotA)>min)) new_hop_distances.put(t.robotA, min);
            if(controller.getID()<t.robotB && (!new_hop_distances.containsKey(t.robotB) || new_hop_distances.get(t.robotB)>min)) new_hop_distances.put(t.robotB, min);
            if(controller.getID()<t.robotC && (!new_hop_distances.containsKey(t.robotC) || new_hop_distances.get(t.robotC)>min)) new_hop_distances.put(t.robotC, min);
        }
        this.frontier_pheromone = new_hop_distances;
    }

    public int getEvasionPheromone(DOTRobotController controller, int robotA, int robotB){
        int correspondRobotID = Math.min(robotA, robotB);
        if(correspondRobotID==controller.getID()){
            if(!this.evasion_pheromone.containsKey(Math.max(robotA, robotB))) return Integer.MAX_VALUE;
            return this.evasion_pheromone.get(Math.max(robotA, robotB));
        }
        Neighbor correspondingNeighbor = controller.neighborManager.getNeighbor(correspondRobotID);
        if(correspondingNeighbor==null){
            //System.err.println(controller.getID()+": PublicVariables.getEvasionPheromone didn't find neighbor, returning Integer.MAX_VALUE");
            return  Integer.MAX_VALUE;}
        if(!correspondingNeighbor.publicVariables.evasion_pheromone.containsKey(Math.max(robotA, robotB))) return  Integer.MAX_VALUE;
        return correspondingNeighbor.publicVariables.evasion_pheromone.get(Math.max(robotA, robotB));
    }

    public void update_evasion_pheromone(DOTRobotController controller){
        if(!this.isStatic){ evasion_pheromone.clear(); return;}
        HashMap<Integer, Integer> tmpMap = new HashMap<Integer, Integer>();

        //Get all triangles that are occupied
        ArrayList<PublicVariables.Triangle> triangles = new ArrayList<PublicVariables.Triangle>();
        for(Neighbor n: controller.neighborManager.getNeighbors()){
            if(n.publicVariables.in_triangle!=null && !triangles.contains(n.publicVariables.in_triangle)){
                triangles.add(n.publicVariables.in_triangle);
            }
        }

        //Set all edges that are in an not occupied triangle to zero
        for(PublicVariables.Triangle t: this.triangles){
            if(!triangles.contains(t)){
                if(controller.getID()<t.robotA) tmpMap.put(t.robotA, 0);
                if(controller.getID()<t.robotB) tmpMap.put(t.robotB, 0);
                if(controller.getID()<t.robotC) tmpMap.put(t.robotC, 0);
            }
        }

        //Inductively update the rest
        for(PublicVariables.Triangle t: this.triangles){
            int ab = getEvasionPheromone(controller, t.robotA, t.robotB);
            int bc = getEvasionPheromone(controller, t.robotB, t.robotC);
            int ca = getEvasionPheromone(controller, t.robotC, t.robotA);
            int min = Math.min(ab,Math.min(bc,ca));
            if(min!=Integer.MAX_VALUE) min+=1;
            if(controller.getID()<t.robotA && (!tmpMap.containsKey(t.robotA) || tmpMap.get(t.robotA)>min)) tmpMap.put(t.robotA, min);
            if(controller.getID()<t.robotB && (!tmpMap.containsKey(t.robotB) || tmpMap.get(t.robotB)>min)) tmpMap.put(t.robotB, min);
            if(controller.getID()<t.robotC && (!tmpMap.containsKey(t.robotC) || tmpMap.get(t.robotC)>min)) tmpMap.put(t.robotC, min);
        }
        this.evasion_pheromone = tmpMap;
    }


}

