package projects.triangulation.Tasks;

import platypus3000.simulation.control.RobotInterface;
import platypus3000.simulation.neighborhood.NeighborView;
import projects.triangulation.*;
import projects.triangulation.Messages.ExpansionTryMsg;
import projects.triangulation.Messages.NewTriangleMessage;
import projects.triangulation.Messages.WallMessage;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.PublicVariables;

/**
 * Created by doms on 1/9/16.
 */
public class EdgeExpansion implements TaskState{
    LocalizationTriangle.Edge edge;

    LocalizationTriangle localizationTriangle;
    TriangleMovementController movementController;

    ExtensionPointOptimization extensionPointOptimization;

    private static final float TRIANGLE_LEFT_TRIGGER = -0.08f;
    private static final float IN_TRIANGLE_TRIGGER = 0.03f;
    private static final float MIN_TRIANGLE_QUALITY = 0.3f;
    private static final float REACTION_DISTANCE = ExtensionPointOptimization.OPT_EDGE_LENGTH;
    private static final float FORCED_REACTION_DISTANCE = 0.15f;

    CheckCandidate checkCandidate = null;

    public EdgeExpansion(RobotInterface robot, DOTRobotController controller, TriangleMovementController tpm, LocalizationTriangle lt, LocalizationTriangle.Edge edge){
        localizationTriangle = lt.clone();
        this.edge = edge;
        movementController = tpm;
        extensionPointOptimization = new ExtensionPointOptimization(controller, edge);
        robot.send(new ExpansionTryMsg(edge.firstRobot, edge.secondRobot), Math.min(edge.firstRobot, edge.secondRobot));
    }

    public boolean loop(RobotInterface robot, DOTRobotController controller){

        try {
            extensionPointOptimization.loop(controller);
            if(checkCandidate!=null){
                if(checkCandidate.loop(robot, controller)){
                    checkCandidate=null;
                }
                return false;
            }

            movementController.setGoalPosition(extensionPointOptimization.getBestPos());

            //How far is the robot in the expansion
            controller.ownPublicVariables.expansion_value = TriangleUtils.squaredDistanceToLineSegment(localizationTriangle.getPosOfTriangleRobot(edge.firstRobot),
                    localizationTriangle.getPosOfTriangleRobot(edge.secondRobot), movementController.getLastPosition());
            //Retreating if there is some other robot
            for(Neighbor nbr: controller.neighborManager.getNeighbors()){
                if(nbr.publicVariables.expansion_value!=null){
                    if(nbr.distance<REACTION_DISTANCE){
                        if(nbr.distance<FORCED_REACTION_DISTANCE  || nbr.publicVariables.expansion_value>controller.ownPublicVariables.expansion_value){
                            robot.say("Retreat");
                            return true;
                        }
                    }
                }
            }

            if(!controller.neighborManager.contains(edge.firstRobot) || !controller.neighborManager.contains(edge.secondRobot)){
                robot.say("Lost connection to edge");
                return true;
            }

            //If a high quality triangle has been built -> add triangle and become static
            if(movementController.reachedGoal()){
                if(extensionPointOptimization.getRating()>MIN_TRIANGLE_QUALITY){
                    if(LocalizationTriangle.findTriangles(controller, TRIANGLE_LEFT_TRIGGER).isEmpty()) {
                        PublicVariables.Triangle newTriangle = new PublicVariables.Triangle(robot.getID(), edge.secondRobot, edge.firstRobot);
                        robot.send(new NewTriangleMessage(newTriangle), edge.firstRobot);
                        robot.send(new NewTriangleMessage(newTriangle), edge.secondRobot);
                        controller.ownPublicVariables.isStatic = true;
                        controller.ownPublicVariables.triangles.add(newTriangle);
                    }
                    return true;
                } else {
                    robot.say("Bad Rating "+extensionPointOptimization.getRating());
                    //controller.addTask(new CheckCandidate(localizationTriangle, edge));
                    return true;
                }
            }

            if(controller.ownPublicVariables.getFrontierPheromone(controller, edge.firstRobot, edge.secondRobot)!=0) return true; //The edge is no longer a frontier edge

            //Old Triangle Left
            if(localizationTriangle.fuzzyContains(controller)<TRIANGLE_LEFT_TRIGGER){
                if(!LocalizationTriangle.findTriangles(controller, IN_TRIANGLE_TRIGGER).isEmpty()){
                    return true; //There seems to be another triangle
                }

                if(CheckCandidate.existsCandidate(controller, edge)){
                    checkCandidate = new CheckCandidate(localizationTriangle, edge);
                    //controller.addTask(new CheckCandidate(localizationTriangle, edge));
                    return false;
                } else {
                    if(extensionPointOptimization.getRating()<MIN_TRIANGLE_QUALITY){ robot.say("No reason to try further"); return true;} //Abort because there is no good triangle possible
                }

            }




        if(robot.hasCollision()){
            //Robot-Robot-Collision
            for(NeighborView n: robot.getNeighborhood()){
                if(n.getDistance()<0.13f){
                    controller.addTask(new SolveCollisionTask());
                    return true;
                }
            }

            //Robot-Wall-Collision
            LocalizationTriangle lt = LocalizationTriangle.create(edge.secondRobot, edge.firstRobot, robot.getID(), controller);
            if(lt==null) return true;
            if(localizationTriangle.fuzzyContains(controller)>-0.2 || ExtensionPointOptimization.rateTriangle(lt,controller)<MIN_TRIANGLE_QUALITY) { //Mark as wall
                robot.send(new WallMessage(edge.firstRobot, edge.secondRobot), edge.firstRobot);
                robot.send(new WallMessage(edge.firstRobot, edge.secondRobot), edge.secondRobot);
                return true;
            } else { //Built triangle as it is good enough
                PublicVariables.Triangle newTriangle = new PublicVariables.Triangle(robot.getID(), edge.secondRobot, edge.firstRobot);
                robot.send(new NewTriangleMessage(newTriangle), edge.firstRobot);
                robot.send(new NewTriangleMessage(newTriangle), edge.secondRobot);
                controller.ownPublicVariables.isStatic = true;
                controller.ownPublicVariables.triangles.add(newTriangle);
                return true;
            }
        }

        return false;
        } catch (TriangleMovementController.MovementControllerException e) {
            System.err.println(robot.getID()+": EdgeExpansion failed due to lost localization triangle in TMC");
            return true;
        } catch (Exception e){
            e.printStackTrace();
            return true;
        }
    }
}

