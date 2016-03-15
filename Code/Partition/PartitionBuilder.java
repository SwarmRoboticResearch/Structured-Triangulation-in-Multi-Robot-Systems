package projects.triangulation.Partition;

import org.jbox2d.common.Vec2;
import platypus3000.analyticstools.overlays.DiscreteStateColorOverlay;
import platypus3000.analyticstools.overlays.VectorOverlay;
import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.PublicVariables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

/**
 * This class provides a method to construct partitions in the triangulation from which we can remove robots as soon as they are stable.
 * This creates holes of limited size (or a street network where the distance to the next road is limited).
 *
 * @author Dominik Krupke
 */
public class PartitionBuilder {
    public static final int MAX_RADIUS = 2; //The radius (in hop distance) such a partition is allowed to have (from the root robot)

    //Debugging
    DiscreteStateColorOverlay marginOverlay;
    VectorOverlay treeOverlay_unstable, treeOverlay_stable;
    Vec2 parent_stable = new Vec2();
    Vec2 parent_unstable = new Vec2();

    public PartitionBuilder(DOTRobotController controller) {
        //Debugging
        marginOverlay = new DiscreteStateColorOverlay(controller, "PartitionMargin", new String[]{"Interior", "Margin", "Undefined"});
        treeOverlay_unstable = new VectorOverlay(controller, "Cluster-unstable", parent_unstable);
        treeOverlay_stable = new VectorOverlay(controller, "Cluster-stable", parent_stable);
    }

    //The size of the subtree in the partition
    private int calcSize(DOTRobotController controller) {
        int size = 1;
        for (Neighbor n : controller.neighborManager.getNeighbors()) {
            if (n.publicVariables.isStatic && n.publicVariables.partition_parent != null && n.publicVariables.partition_parent == controller.getID()) {
                size += n.publicVariables.partition_subsize;
            }
        }
        return size;
    }

    //Checks if the partition is stable in the subtree
    boolean isStable(DOTRobotController controller) {
        //if(controller.ownPublicVariables.partition_stable) return true;
        if (controller.ownPublicVariables.isFrontierOrWall()) return false;
        if (controller.ownPublicVariables.partition_root == null) return false;
        if (controller.ownPublicVariables.frontier_distance <= MAX_RADIUS) return false;
        if(controller.ownPublicVariables.partition_hops==MAX_RADIUS) return true;
        for (Neighbor n : controller.neighborManager.getNeighbors()) {
            if(!isInteriorRobot(n)) continue;
            if(n.publicVariables.partition_frozen) continue;
            if(n.publicVariables.partition_root == null) return false;
            if(n.publicVariables.partition_root.equals(controller.ownPublicVariables.partition_root) && n.publicVariables.partition_hops>controller.ownPublicVariables.partition_hops && !n.publicVariables.partition_stable) return false;
            if(n.publicVariables.partition_parent !=null && n.publicVariables.partition_parent == controller.getID() && !n.publicVariables.partition_stable) return false;
        }
        return true;
    }

    boolean isInteriorRobot(Neighbor n){
        if(!n.publicVariables.isStatic) return false;
        if(n.publicVariables.isFrontierOrWall()) return false;
        return true;
    }

    boolean isInteriorRobot(DOTRobotController r){
        if(!r.ownPublicVariables.isStatic) return false;
        if(r.ownPublicVariables.isFrontierOrWall()) return false;
        return true;
    }

    //The loop to be executed in every iteration from the robots
    public void loop(RobotInterface robot, DOTRobotController controller) {
        //Debugging
        parent_stable.setZero();
        parent_unstable.setZero();
        try {


          //  if(controller.ownPublicVariables.partition_margin) return;
            //Abort if not interior robot
            if(!controller.ownPublicVariables.partition_frozen && !isInteriorRobot(controller)){
                controller.ownPublicVariables.deletePartitionVariables();
                if(controller.ownPublicVariables.isFrontier(controller)) controller.ownPublicVariables.frontier_distance = 0;
                robot.say("-");
                return;
            }
            robot.say("");

            //Get all neighbors in the triangulation
            ArrayList<Neighbor> triangleNeighbors = new ArrayList<Neighbor>();
            for (PublicVariables.Triangle t : controller.ownPublicVariables.triangles) {
                if (t.robotA != controller.getID()) {
                    Neighbor n = controller.neighborManager.getNeighbor(t.robotA);
                    if (!triangleNeighbors.contains(n)) triangleNeighbors.add(n);
                }
                if (t.robotB != controller.getID()) {
                    Neighbor n = controller.neighborManager.getNeighbor(t.robotB);
                    if (!triangleNeighbors.contains(n)) triangleNeighbors.add(n);
                }
                if (t.robotC != controller.getID()) {
                    Neighbor n = controller.neighborManager.getNeighbor(t.robotC);
                    if (!triangleNeighbors.contains(n)) triangleNeighbors.add(n);
                }
            }

            //frontier distance
            controller.ownPublicVariables.frontier_distance = Integer.MAX_VALUE;
            for(Neighbor nbr: triangleNeighbors){
                if(nbr.publicVariables.frontier_distance<controller.ownPublicVariables.frontier_distance) controller.ownPublicVariables.frontier_distance = nbr.publicVariables.frontier_distance+1;
            }


            //Check if ready for freeing robot
            if(controller.ownPublicVariables.partition_frozen){
                for(Neighbor nbr: triangleNeighbors){
                    if(nbr.publicVariables.partition_root==null || !nbr.publicVariables.partition_root.equals(controller.ownPublicVariables.partition_root)){ //Margin
                        controller.ownPublicVariables.partition_margin = true;
                        robot.say("Margin "+(nbr.publicVariables.partition_root==null));
                        //TODO: Check if all margin -> reset partition
                        return;
                    }
                    if(!nbr.publicVariables.partition_frozen || (nbr.publicVariables.partition_parent!=null && nbr.publicVariables.partition_parent.equals(controller.getID()) && !nbr.publicVariables.partition_margin)){
                        robot.say("Waiting");
                        return;
                    }
                }
                controller.delete(robot, "Partition");
                return;
            }

            //Determine sizes of clusters
            HashMap<Integer, Integer> cluster_sizes = new HashMap<Integer, Integer>();
            for (Neighbor nbr : triangleNeighbors) {
                if(nbr.publicVariables.partition_root==null) continue;
                if(cluster_sizes.containsKey(nbr.publicVariables.partition_root)){
                    if(cluster_sizes.get(nbr.publicVariables.partition_root)<nbr.publicVariables.partition_supersize){
                        cluster_sizes.put(nbr.publicVariables.partition_root,nbr.publicVariables.partition_supersize);
                    }
                } else {
                    cluster_sizes.put(nbr.publicVariables.partition_root,nbr.publicVariables.partition_supersize);
                }
            }

            ArrayList<Neighbor> candidates = new ArrayList<Neighbor>();
            if(controller.ownPublicVariables.partition_stable && controller.neighborManager.contains(controller.ownPublicVariables.partition_parent) &&
                    !controller.neighborManager.getNeighbor(controller.ownPublicVariables.partition_parent).publicVariables.partition_root.equals(controller.ownPublicVariables.partition_root)){
                candidates.add(controller.neighborManager.getNeighbor(controller.ownPublicVariables.partition_parent));
            } else {
                for (Neighbor nbr : triangleNeighbors) {
                    if(nbr.publicVariables.partition_hops>=MAX_RADIUS || nbr.publicVariables.partition_root==null) continue;
                    candidates.add(nbr);
                }
            }

            //Estimate max size
            int max_size = 0;
            for(Neighbor nbr: candidates){
                int size = cluster_sizes.get(nbr.publicVariables.partition_root);
                if(size>max_size) max_size = size;
            }

            //Only keep robots from clusters with max size
            ArrayList<Neighbor> candidates_tmp = new ArrayList<Neighbor>();
            for(Neighbor nbr: candidates){
                if(cluster_sizes.get(nbr.publicVariables.partition_root)==max_size) candidates_tmp.add(nbr);
            }
            candidates = candidates_tmp;

            // Estimate min hop
            int min_hops = MAX_RADIUS;
            for(Neighbor nbr: candidates){
                int hops = nbr.publicVariables.partition_hops;
                if(hops<min_hops) min_hops = hops;
            }

            // Only keep those with min hops
            candidates_tmp = new ArrayList<Neighbor>();
            for(Neighbor nbr: candidates){
                if(nbr.publicVariables.partition_hops==min_hops) candidates_tmp.add(nbr);
            }
            candidates = candidates_tmp;

            //Min cluster id
            int min_id = Integer.MAX_VALUE;
            for(Neighbor nbr: candidates){
                int id = nbr.publicVariables.partition_root;
                if(id<min_id) min_id = id;
            }

            // Only keep those with min cluster id
            candidates_tmp = new ArrayList<Neighbor>();
            for(Neighbor nbr: candidates){
                if(nbr.publicVariables.partition_root==min_id) candidates_tmp.add(nbr);
            }
            candidates = candidates_tmp;

            //Of the remaining candidates, the one with min id
            Neighbor best_parent = null;
            for(Neighbor nbr: candidates){
                if(best_parent==null || nbr.ID<best_parent.ID) best_parent = nbr;
            }


            if(best_parent==null && controller.ownPublicVariables.partition_hops>0){//become root?

                controller.ownPublicVariables.deletePartitionVariables();
                controller.ownPublicVariables.partition_root = new Random().nextInt();
                controller.ownPublicVariables.partition_parent = null;
                controller.ownPublicVariables.partition_subsize = 1;
                controller.ownPublicVariables.partition_supersize = 1;
                controller.ownPublicVariables.partition_hops = 0;
                controller.ownPublicVariables.partition_stable = false;
                controller.ownPublicVariables.partition_frozen = false;
            } else if(controller.ownPublicVariables.partition_hops == 0 &&
                    (best_parent ==null || controller.ownPublicVariables.partition_root.equals(best_parent.publicVariables.partition_root) ||
                            controller.ownPublicVariables.partition_supersize>best_parent.publicVariables.partition_supersize ||
                            (controller.ownPublicVariables.partition_supersize==best_parent.publicVariables.partition_supersize &&
                                    controller.ownPublicVariables.partition_root < best_parent.publicVariables.partition_root))){//is root?
                controller.ownPublicVariables.partition_stable = isStable(controller);
                controller.ownPublicVariables.partition_frozen = controller.ownPublicVariables.partition_stable;
                controller.ownPublicVariables.partition_subsize = calcSize(controller);
                controller.ownPublicVariables.partition_supersize = controller.ownPublicVariables.partition_subsize;
                assert controller.ownPublicVariables.partition_parent == null;
            } else {
                assert best_parent != null; //Otherwise it would be root

                controller.ownPublicVariables.partition_hops = best_parent.publicVariables.partition_hops + 1;
                controller.ownPublicVariables.partition_root = best_parent.publicVariables.partition_root;
                controller.ownPublicVariables.partition_parent = best_parent.ID;
                controller.ownPublicVariables.partition_stable = isStable(controller);
                controller.ownPublicVariables.partition_frozen = best_parent.publicVariables.partition_frozen;
                controller.ownPublicVariables.partition_subsize = calcSize(controller);
                controller.ownPublicVariables.partition_supersize = best_parent.publicVariables.partition_supersize;
            }


            //Debugging
            //robot.say((best_parent!=null)+"|"+controller.ownPublicVariables.partition_margin+"|"+controller.ownPublicVariables.partition_hops + "|" + controller.ownPublicVariables.partition_root + "|" + controller.ownPublicVariables.partition_frozen+"|"+ controller.ownPublicVariables.partition_stable+"|"+controller.ownPublicVariables.partition_supersize);
            if(controller.ownPublicVariables.partition_parent!=null){
                if(controller.ownPublicVariables.partition_stable) parent_stable.set(robot.getNeighborhood().getById(controller.ownPublicVariables.partition_parent).getLocalPosition());
                else parent_unstable.set(robot.getNeighborhood().getById(controller.ownPublicVariables.partition_parent).getLocalPosition());
            }
        } catch(Exception e){
            //is_root = false;
            controller.ownPublicVariables.deletePartitionVariables();
            e.printStackTrace();
            System.err.println("Error in PartitionBuilder in robot "+controller.getID()+"! Resetting this component...");
        }
    }
}
