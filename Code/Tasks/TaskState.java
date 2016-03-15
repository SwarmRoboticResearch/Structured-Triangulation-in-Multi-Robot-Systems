package projects.triangulation.Tasks;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;

/**
 * Created by doms on 1/26/16.
 */
public interface TaskState {
    public boolean loop(RobotInterface robot, DOTRobotController controller);
}
