package projects.triangulation.Tasks;

import org.jbox2d.common.Vec2;
import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;
import projects.triangulation.LocalizationTriangle;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.PublicVariables;
import projects.triangulation.TriangleMovementController;

import java.util.ArrayList;

/**
 * The task of this class is to remove robots from a triangle if there are multiple robots in the same triangle.
 * To prevent inconsistencies and collisions, we only want one robot per triangle.
 * If there are more than one, we remove the robot that is closest to an edge that has a minimal distance to a free triangle.
 */
public class Evasion implements TaskState {


    public Evasion() {
    }

    //Returns all candidate edges that have shortest distance to a free triangle
    private ArrayList<LocalizationTriangle.Edge> getMinEdges(DOTRobotController controller, RobotInterface robot, LocalizationTriangle t) {
        controller.movementController.changeLocalizationTriangle(t);
        int ab = controller.ownPublicVariables.getEvasionPheromone(controller, t.robotA, t.robotB);
        int bc = controller.ownPublicVariables.getEvasionPheromone(controller, t.robotB, t.robotC);
        int ca = controller.ownPublicVariables.getEvasionPheromone(controller, t.robotC, t.robotA);
        int min = Math.min(ab, Math.min(bc, ca));
        ArrayList<LocalizationTriangle.Edge> minEdges = new ArrayList<LocalizationTriangle.Edge>();
        if (ab == min) minEdges.add(t.getEdgeAB());
        if (bc == min) minEdges.add(t.getEdgeBC());
        if (ca == min) minEdges.add(t.getEdgeCA());
        return minEdges;
    }

    int max_time = 1000;
    @Override
    public boolean loop(RobotInterface robot, DOTRobotController controller) {
        if(max_time--<0){//Prevent deadlocks
            controller.movementController.resetGoal();
            controller.movementController.changeLocalizationTriangle(null);
            controller.ownPublicVariables.reset();
            return true;
        }
        if (robot.hasCollision()) {
            controller.addTask(new SolveCollisionTask());
            return true;
        } //If collision -> solve it first

        if (controller.movementController.getLocalizationTriangle() == null) {
            controller.ownPublicVariables.in_triangle = null;
            System.err.println("ROBOT-" + robot.getID() + " called evasion without a valid triangle");
            return true;
        }
        try {
            //Get the positions of all neighbors in the triangle
            ArrayList<Vec2> nbrs = new ArrayList<Vec2>();
            PublicVariables.Triangle ownTriangle = controller.movementController.getLocalizationTriangle().triangle();
            controller.ownPublicVariables.in_triangle = ownTriangle;
            for (Neighbor nbr : controller.neighborManager.getNeighbors()) {
                if (nbr.publicVariables.in_triangle != null && nbr.publicVariables.in_triangle.equals(ownTriangle)) {
                    robot.say("alone");
                    Vec2 pos = controller.movementController.getLocalizationTriangle().locateNbr(controller, nbr.ID);
                    if (pos != null) nbrs.add(pos);
                }
            }

            if (nbrs.isEmpty()) { //If the robot is alone, go to the center of the triangle and end this task
                controller.movementController.move_into_triangle(controller);
                if (controller.movementController.reachedGoal()) return true;
                return false;
            } else { //If the robot is not alone, check if it is the one closest to an edge with minimal evasion pheromone value and if it is, leave

                //Calculate the distance of the neighbors to their closest exit edges as well as the own closest exit edge
                float minDist = Float.MAX_VALUE;
                float ownMinDist = Float.MAX_VALUE;
                LocalizationTriangle.Edge bestEdge = null;
                Vec2 pos = controller.movementController.locate(controller);
                if (pos == null) {
                    System.err.println("ROBOT-" + robot.getID() + " localization failed -> abort evasion");
                    return true;
                } //Abort -> Localization failed
                for (LocalizationTriangle.Edge e : getMinEdges(controller, robot, controller.movementController.getLocalizationTriangle())) {
                    for (Vec2 n : nbrs) {
                        float d = e.getCenter().sub(n).lengthSquared();
                        if (d < minDist) minDist = d;
                    }

                    float d2 = pos.sub(e.getCenter()).lengthSquared();
                    if (bestEdge == null || ownMinDist > d2) {
                        bestEdge = e;
                        ownMinDist = d2;
                    }
                }

                if (ownMinDist < minDist * 1.05f) { //If the robot is closest (with some tolerance) to an exit edge -> leave it via this edge
                    controller.movementController.setGoalPosition(bestEdge.getCenter());
                    if (controller.movementController.reachedGoal()) { //As soon as the center of the exit edge is reached, swap triangle
                        LocalizationTriangle lt = controller.movementController.getLocalizationTriangle().getAdjacentTriangle(controller, bestEdge);
                        controller.movementController.changeLocalizationTriangle(lt);
                        if(lt==null) {
                            System.err.println(robot.getID()+" evasion -> no adjacent triangle");
                            return true;
                        }
                    }
                } else { //If it is not the closest robot -> simply wait
                    controller.movementController.stopMoving();
                }
            }
            return false;
        } catch (TriangleMovementController.MovementControllerException e) {
            e.printStackTrace();
            return true;
        }
    }
}
