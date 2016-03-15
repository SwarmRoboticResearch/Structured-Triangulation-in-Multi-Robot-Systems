package projects.triangulation.Tasks;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;
import projects.triangulation.LocalizationTriangle;
import projects.triangulation.NeighborManager.Neighbor;

import java.util.Random;

/**
 * This class implements the lost state of a free robot. It finds a reference triangle (LocalizationTriangle) via random walk.
 */
public class FindTriangleTask implements TaskState{
    public boolean loop(RobotInterface robot, DOTRobotController controller){
        controller.movementController.resetGoal();//If there are any goals saved in the movement controller, forget them
        if(LocalizationTriangle.getClosestTriangle(controller)!=null) return true; //Triangle has been found -> Reached goal -> stop
        if(robot.hasCollision()) {controller.addTask(new SolveCollisionTask()); return false; } //Collisions need special treatment (in especially shorter straight movements)
        // Try to approach a static robot but if this is not possible do a random movement
        if (!approach_single_robot(controller)) {
            reset_approach_single_robot();
            randomMovement(robot, controller);
        }
        return false;
    }

    /**********************************************************************************
     * One static robot in range -> Random walk with adapted probabilities
     **********************************************************************************/

    //This variables save the scheduled movement
    int approach_move_for = 0;
    int approach_rotate_for = 0;
    boolean approach_ccw = false;

    Neighbor approach_robot = null; //The robot that is tried to approached
    Float last_dist = null; //Used for checking if the last movement decreased or increased the distance

    boolean approach_single_robot(DOTRobotController controller) {
        Random rand = new Random();//The random number generator

        // Find or change robot to approach, if there is no such -> return false
        if (approach_robot == null || rand.nextInt(100) == 1 || !controller.neighborManager.contains(approach_robot.ID)) {
            Neighbor minNbr = null;
            for (Neighbor tps : controller.neighborManager.getNeighbors()) {
                if (tps.publicVariables.isStatic) {
                    if ((minNbr == null || tps.distance < minNbr.distance)) {
                        minNbr = tps;
                    }
                }
            }
            if (minNbr == null) return false; // This method could not be executed
            approach_robot = minNbr;
        }

        // Continue a previous scheduled movement or program a new one
        if (approach_rotate_for > 0) {
            controller.movementController.rotate(approach_ccw);
            approach_rotate_for--;
        } else if (approach_move_for > 0) {
            controller.movementController.moveForwards();
            approach_move_for--;
        } else {
            if (last_dist == null) {
                last_dist = approach_robot.distance;
            } else {
                float new_dist = approach_robot.distance;
                if (new_dist < last_dist) {
                    if (rand.nextInt(10) == 1) {
                        approach_rotate_for = rand.nextInt(10);
                        approach_move_for = rand.nextInt(10);
                    } else {
                        approach_move_for = 10;
                    }
                } else {
                    approach_rotate_for = rand.nextInt(10);
                    approach_move_for = rand.nextInt(10);
                }
                last_dist = new_dist;
            }
        }
        return true; //This method could be executed but is not necessarily done
    }

    //Resets previous scheduled movements
    void reset_approach_single_robot() {
        last_dist = null;
        approach_move_for = 0;
        approach_rotate_for = 0;
        approach_ccw = false;
        approach_robot = null;
    }

    /**********************************************************************************
     * No static robot in range -> Random Walk
     **********************************************************************************/

    //Random movement to find another robot
    int random_move_for = 0;
    int random_rotate_for = 0;
    boolean random_ccw = false;

    void randomMovement(RobotInterface robot, DOTRobotController controller) {
        //If not scheduled movement to continue, schedule a new one
        if (random_move_for == 0 && random_rotate_for == 0) {
            Random rand = new Random();
            if (rand.nextBoolean()) {
                random_move_for = rand.nextInt(100);
            } else {
                random_rotate_for = rand.nextInt(30);
                random_ccw = rand.nextBoolean();
            }
        }

        //execute scheduled movement
        if (random_move_for > 0) {
            controller.movementController.moveForwards();
            random_move_for--;
        } else if (random_rotate_for > 0) {
            controller.movementController.rotate(random_ccw);
            random_rotate_for--;
        }
    }

    void resetRandomMovement() {
        random_ccw = false;
        random_move_for = 0;
        random_rotate_for = 0;
    }
}
