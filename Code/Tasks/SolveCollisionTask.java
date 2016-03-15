package projects.triangulation.Tasks;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;

import java.util.Random;

/**
 * The task of this class is to solve a collision such that we again have controlled movement.
 * During a collision we cannot do orientation estimations thus we have to rotate randomly and move forward to check
 * if the collision is solved.
 */
public class SolveCollisionTask implements TaskState {

    @Override
    public boolean loop(RobotInterface robot, DOTRobotController controller) {
        if(!robot.hasCollision()) return true; //finished
        controller.movementController.changeLocalizationTriangle(null);
        //System.out.println("Collision");
        controller.movementController.resetGoal();
        //If not executing something, create a new random movement task
        if (collision_move_for == 0 && collision_rotate_for == 0) {
            Random rand = new Random();
            if (rand.nextBoolean()) {
                collision_move_for = rand.nextInt(10);
            } else {
                collision_rotate_for = rand.nextInt(20);
                //collision_ccw = rand.nextBoolean();
            }
        }

        //execute random movement task
        if (collision_move_for > 0) {
            controller.movementController.moveForwards();
            collision_move_for--;
        } else if (collision_rotate_for > 0) {
            controller.movementController.rotate(collision_ccw);
            collision_rotate_for--;
        }
        return false;
    }

    int collision_move_for = 0;
    int collision_rotate_for = 0;
    boolean collision_ccw = new Random().nextBoolean();

}

