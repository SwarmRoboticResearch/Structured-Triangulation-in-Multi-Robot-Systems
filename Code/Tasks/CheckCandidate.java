package projects.triangulation.Tasks;

import org.jbox2d.common.Vec2;
import platypus3000.simulation.control.RobotInterface;
import platypus3000.utils.AngleUtils;
import projects.triangulation.DOTRobotController;
import projects.triangulation.ExtensionPointOptimization;
import projects.triangulation.LocalizationTriangle;
import projects.triangulation.Messages.NewTriangleMessage;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.PublicVariables;
import projects.triangulation.TriangleMovementController;

/**
 * Created by doms on 2/1/16.
 */
public class CheckCandidate implements TaskState {
    private static final float TRIANGLE_LEFT_TRIGGER = -0.1f;
    private static final float IN_TRIANGLE_TRIGGER = 0.03f;
    private static final float MIN_TRIANGLE_RATING = 0.3f;
    LocalizationTriangle.Edge edge;
    LocalizationTriangle lt;

    public CheckCandidate(LocalizationTriangle lt, LocalizationTriangle.Edge edge) {
        this.edge = edge;
        this.lt = lt;
    }

    static private LocalizationTriangle getBestCandidate(DOTRobotController controller, LocalizationTriangle.Edge edge){
        LocalizationTriangle best_lt = null;
        float best_lt_rating = 0;
        for (Neighbor nbr : controller.neighborManager.getNeighbors()) {
            if (edge.triangle.containsTriangleRobot(nbr.ID)) continue;
            if (nbr.publicVariables.isStatic) {
                Vec2 pos = edge.triangle.locateNbr(controller, nbr.ID);
                if(pos!=null){
                    if (AngleUtils.normalizeToMinusPi_Pi(AngleUtils.getClockwiseRadian(edge.triangle.getPosOfTriangleRobot(edge.secondRobot).sub(pos), edge.triangle.getPosOfTriangleRobot(edge.firstRobot).sub(pos))) > 0) continue;
                }
                LocalizationTriangle lt2 = LocalizationTriangle.create(edge.secondRobot, edge.firstRobot, nbr.ID, controller);
                if (lt2 == null) continue;
                if (lt2.fuzzyContains(controller) < TRIANGLE_LEFT_TRIGGER) {
                    continue;
                }
                float rating = ExtensionPointOptimization.rateTriangle(lt2,controller);
                if (best_lt == null || rating > best_lt_rating) {
                    best_lt = lt2;
                    best_lt_rating = rating;
                }
            }
        }
        return best_lt;
    }

    public static boolean existsCandidate(DOTRobotController controller, LocalizationTriangle.Edge edge) {
        LocalizationTriangle best_lt = getBestCandidate(controller, edge);
        if(best_lt==null) return false;
        return ExtensionPointOptimization.rateTriangle(best_lt, controller) >= MIN_TRIANGLE_RATING; //More conservative
    }

    @Override
    public boolean loop(RobotInterface robot, DOTRobotController controller) {
        try {
            if (lt.fuzzyContains(controller) > TRIANGLE_LEFT_TRIGGER) {
                controller.movementController.setGoalPosition(edge.getExtensionPoint());
                assert !controller.movementController.reachedGoal();
                return false;
            }

            LocalizationTriangle best_lt = getBestCandidate(controller, edge);

            if (best_lt == null || ExtensionPointOptimization.rateTriangle(best_lt, controller) < MIN_TRIANGLE_RATING) {
               // if(best_lt==null) System.err.println(robot.getID()+": CheckCandidate failed with no candidate");
                //else System.err.println(robot.getID()+":CheckCandidate failed with bad rating "+ ExtensionPointOptimization.rateTriangle(best_lt, controller));
                robot.say((best_lt == null ? "No valid candidate" : "Bad rating " + best_lt));
                return true;
            }
            controller.movementController.setGoalPosition(lt.transformFrom(best_lt, best_lt.getCenter(), controller));
            if (controller.movementController.reachedGoal()) {
                if (LocalizationTriangle.findTriangles(controller, TRIANGLE_LEFT_TRIGGER).isEmpty()) {
                    PublicVariables.Triangle t = new PublicVariables.Triangle(best_lt.robotA, best_lt.robotB, best_lt.robotC);
                    robot.send(new NewTriangleMessage(t), best_lt.robotA);
                    robot.send(new NewTriangleMessage(t), best_lt.robotB);
                    robot.send(new NewTriangleMessage(t), best_lt.robotC);
                    return true;
                } else {
                    //System.err.println(robot.getID()+":CheckCandidate failed with not empty");
                    robot.say("Overlapping");
                    return true;
                }
            }


            return false;
        } catch (TriangleMovementController.MovementControllerException e) {
            e.printStackTrace();
            return true;
        }
    }
}
