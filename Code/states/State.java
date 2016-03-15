package projects.triangulation.states;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;

/**
 * Created by doms on 1/26/16.
 */
public interface State {
    public void loop(RobotInterface robot, DOTRobotController controller);
}
