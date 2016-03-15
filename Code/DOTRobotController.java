package projects.triangulation;

import platypus3000.analyticstools.overlays.DiscreteStateColorOverlay;
import platypus3000.simulation.communication.Message;
import platypus3000.simulation.control.RobotController;
import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.Messages.*;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.NeighborManager;
import projects.triangulation.NeighborManager.PublicVariables;
import projects.triangulation.Overlays.EvasionPheromoneOverlay;
import projects.triangulation.Overlays.TriangulationOverlay;
import projects.triangulation.Partition.PartitionBuilder;
import projects.triangulation.Tasks.*;

import java.util.*;

/**
 * This is the basic class for the triangulation. It controls the robot and manages its tasks. The tasks itself (like finding the frontier or solving a collision) are
 * implemented in extern classes but still called by this one.
 *
 * @author Dominik Krupke
 * Last modifed: 12. Feb. 2016
 */
public class DOTRobotController extends RobotController {
    public static final boolean PARTITION_MODE = true; //Only triangulation or with partition?
    private int ID; public int getID(){return ID;} //ID of the robot
    private final Stack<TaskState> tasks = new Stack<TaskState>(); //The tasks the robot has to execute (the latest task is executed, the others wait)
    public NeighborManager neighborManager; //Manages the public variables

    public PublicVariables ownPublicVariables; //The own public variable
    public TriangleMovementController movementController; //Allows a controlled movement in an coordinate system build by a static triangle.
    TriangulationOverlay triangulationOverlay = null; //Debugging/Visualization
    EvasionPheromoneOverlay evasionPheromoneOverlay = null; //Debugging/Visualization
    DiscreteStateColorOverlay stateColorOverlay; //Debugging/Visualization

    PartitionBuilder partitionBuilder; //Partition algorithm

    /**
     * Initializes the member variables and objects of this controller.
     * @param robot The assigned robot
     */
    public void init(RobotInterface robot) {
        this.ID = robot.getID();
        if(ownPublicVariables==null) ownPublicVariables = new PublicVariables(robot.getID());
        neighborManager = new NeighborManager(ownPublicVariables);
        triangulationOverlay = new TriangulationOverlay(robot, ownPublicVariables, this, "RobotTriangulation");
        evasionPheromoneOverlay = new EvasionPheromoneOverlay(robot, ownPublicVariables, this, "EvasionPheromone");
        movementController = new TriangleMovementController(this);
        stateColorOverlay = new DiscreteStateColorOverlay(this, "TriangulationState", new String[]{"Static", "FindFrontier", "Expansion", "Evasion", "Lost", "Collision", "CheckCandidate"});
        partitionBuilder = new PartitionBuilder(this);
    }



    /**
     * Removes a static robot from the triangulation. Can of course only called by itself
     * @param robot The robot to be removed.
     */
    public void delete(RobotInterface robot, String reason){
        if(!ownPublicVariables.isStatic){System.err.println("Tried to delete a non-static robot"); return; }
        //assert ownPublicVariables.isStatic;
        robot.send(new DeletionNotification());
        ownPublicVariables.reset();
        expansion_trials.clear();
        System.out.println(robot.getID()+" deleted because: "+reason);
    }

    //Processes the incoming messages (only static robots will receive some)
    private void process_messages(RobotInterface robot) {
        for (Message m : robot.incomingMessages()) {
            if (m.msg instanceof NewTriangleMessage) { //A new triangle has been found/built
                NewTriangleMessage ntm = (NewTriangleMessage) m.msg;
                ownPublicVariables.addTriangle(ntm.triangle);
                m.delete();
            } else if (m.msg instanceof WallMessage) { //A frontier edge cannot be expanded because there is a wall
                WallMessage wm = (WallMessage) m.msg;
                ownPublicVariables.wall_edges.add(Math.max(wm.robotA, wm.robotB));
                m.delete();
            } else if(m.msg instanceof DeletionDemand){ //A robot demands that this robot is deleted because it discovered some inconsistencies
                delete(robot, "DeletionDemand");
                m.delete();
            } else if(m.msg instanceof DeletionNotification){ //A neighbor has be deleted an all its triangles have to be removed
                Stack<PublicVariables.Triangle> toRemove = new Stack<PublicVariables.Triangle>();
                for(PublicVariables.Triangle t: ownPublicVariables.triangles){
                    if(t.contains(m.sender)) toRemove.push(t);
                }
                for(PublicVariables.Triangle t: toRemove){
                    ownPublicVariables.triangles.remove(t);
                }
                if(ownPublicVariables.frontier_pheromone.containsKey(m.sender)) ownPublicVariables.frontier_pheromone.remove((Integer)m.sender);
                if(ownPublicVariables.wall_edges.contains(m.sender)) ownPublicVariables.wall_edges.remove((Integer)m.sender);
                m.delete();
            } else if(m.msg instanceof ExpansionTryMsg){
                ExpansionTryMsg msg = (ExpansionTryMsg) m.msg;
                if(!expansion_trials.containsKey(msg.max)) expansion_trials.put(msg.max, 1);
                else expansion_trials.put(msg.max, expansion_trials.get(msg.max)+1);
                m.delete();
            }
        }
    }

    /**
     * Adds a tasks with highest priority. It will be the only thing the robot executes (except for basic maintenance) until it is finished or a new one is added.
     * It behaves like a stack: Only the latest task is executed and as soon as it finishes, the previous task is continued.
     * @param task The new task
     */
    public void addTask(TaskState task) {
        tasks.push(task);
    }


    public HashMap<Integer, Integer> expansion_trials = new HashMap<Integer, Integer>(); //Saves for static robots how often a frontier edge has already been tried to be expanded
    public ArrayList<Integer> blacklist = new ArrayList<Integer>();
    @Override
    public void loop(RobotInterface robot) {
        try {
            if (!neighborManager.hasStabilized()) { //Gather information before doing anything
                neighborManager.loop_prologue(robot);
                neighborManager.loop_epilogue(robot);
                return;
            }

            neighborManager.loop_prologue(robot); //Update neighbors and their public variables

            if (PARTITION_MODE) { //Build cluster for partition
                partitionBuilder.loop(robot, this);
            }

            //Update the 2-hop neighborhood
            HashMap<Integer, Float> twoHopDistances = new HashMap<Integer, Float>();
            for (Neighbor n : neighborManager.getNeighbors()) {
                twoHopDistances.put(n.ID, n.distance);
            }
            ownPublicVariables.distances = twoHopDistances;

            ownPublicVariables.expansion_value = null;//reset expansion value, will be set every round if not null

            //Update the pheromones,
            ownPublicVariables.update_evasion_pheromone(this);

            //Blacklist frontier edges that have too many failed expansion attempts.
            blacklist.clear();
            if (ownPublicVariables.isStatic) {
                for (Map.Entry<Integer, Integer> e : expansion_trials.entrySet()) {
                    if (e.getValue() > 10){
                        blacklist.add(e.getKey());
                        delete(robot, "Blacklist");
                        robot.send(new DeletionDemand(), e.getKey());
                        break;
                    }
                }
            } else {
                expansion_trials.clear();
            }

            //Frontier Pheromone
            ownPublicVariables.update_frontier_pherome(robot, this, blacklist);

            //Prolog of MovementController (e.g. remove outdated LocalizationTriangles)
            movementController.loop_prologue(robot);

            //Incoming messages (actually only static robots will receive some)
            process_messages(robot);

            if (!tasks.empty()) { //If there is an task -> execute it
                TaskState task = tasks.peek();
                if (task.loop(robot, this)) tasks.remove(task); //Task terminated
            } else { //No task -> basic behavior
                if (ownPublicVariables.isStatic) {
                    ownPublicVariables.in_triangle = null; //A static robot is in no triangle (or at least should not be)

                    //Deactivate and reset the movement controller
                    movementController.stopMoving();
                    movementController.localizationTriangle = null;

                    if (ownPublicVariables.triangles.isEmpty()) delete(robot, "triangles.isEmpty()"); //If it is in no triangle -> remove it

                    //A static robot should not be in any triangle it is not part of
                    for (LocalizationTriangle t : LocalizationTriangle.findTriangles(this, 0)) {
                        if (!t.containsTriangleRobot(robot.getID())) {
                            System.err.println(robot.getID() + ": Static robot in other triangle");
                            if (new Random().nextInt(10) == 0) delete(robot, "Static robot in other triangle");
                        }
                    }
                } else { //If the robot is not static it should find an frontier edge and become static
                    ownPublicVariables.deleteStaticRobotVariables();
                    addTask(new FindFrontier());
                }
            }

            // Update the public variable that states in which triangle a free robot is (for avoiding inconsistencies each triangle should only contain one free robot)
            if (ownPublicVariables.isStatic || movementController.localizationTriangle == null) {
                ownPublicVariables.in_triangle = null;
            } else {
                ownPublicVariables.in_triangle = new PublicVariables.Triangle(movementController.localizationTriangle);
            }


            //No further changes will occur in this round -> let the tools execute their programmed tasks
            movementController.loop_epilogue(robot);
            neighborManager.loop_epilogue(robot);


            //DEBUGGING AND VISUALIZATION
            if (ownPublicVariables.isStatic) { //"Static", "FindFrontier", "Expansion", "Evasion", "Lost", "Collision"
                stateColorOverlay.setState(0);
            } else if (!tasks.isEmpty() && tasks.peek() instanceof FindFrontier) {
                stateColorOverlay.setState(1);
            } else if (!tasks.isEmpty() && tasks.peek() instanceof EdgeExpansion) {
                stateColorOverlay.setState(2);
            } else if (!tasks.isEmpty() && tasks.peek() instanceof Evasion) {
                stateColorOverlay.setState(3);
            } else if (!tasks.isEmpty() && tasks.peek() instanceof FindTriangleTask) {
                stateColorOverlay.setState(4);
            } else if (!tasks.isEmpty() && tasks.peek() instanceof SolveCollisionTask) {
                stateColorOverlay.setState(5);
            } else if (!tasks.isEmpty() && tasks.peek() instanceof CheckCandidate) {
                stateColorOverlay.setState(6);
            }
        } catch(Exception e){
            e.printStackTrace();
            ownPublicVariables.reset();
            tasks.clear();
            System.err.println("Error in robot "+robot.getID()+"! Restarting...");
            movementController.loop_epilogue(robot);
            neighborManager.loop_epilogue(robot);
        }
    }

    @Override
    public void print_debug() {
        System.out.println(ownPublicVariables.toString());
    }

    // ** CONSTRUCTOR **

    //Constructor for the initial static triangle
    public DOTRobotController(int id, PublicVariables.Triangle t, int frontierEdgeA, int frontierEdgeB) {
        ownPublicVariables = new PublicVariables(id);
        ownPublicVariables.addTriangle(t);
        int back_robot = (t.robotA == frontierEdgeA || t.robotA == frontierEdgeB?((t.robotB==frontierEdgeA||t.robotB==frontierEdgeB)?t.robotC:t.robotB):t.robotA);
        ownPublicVariables.isStatic = true;
        if(id!=back_robot) ownPublicVariables.wall_edges.add(back_robot);
        if(id==back_robot){ ownPublicVariables.wall_edges.add(frontierEdgeA); ownPublicVariables.wall_edges.add(frontierEdgeB);}
    }

    //Constructor for a free robot
    public DOTRobotController() {
    }

}



