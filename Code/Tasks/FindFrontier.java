package projects.triangulation.Tasks;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;
import projects.triangulation.LocalizationTriangle;
import projects.triangulation.Messages.DeletionDemand;
import projects.triangulation.NeighborManager.Neighbor;
import projects.triangulation.NeighborManager.PublicVariables;
import projects.triangulation.TriangleMovementController;

import java.util.ArrayList;
import java.util.Random;

/**
 * This class's task is to move the corresponding robot to the closest frontier edge.
 */
public class FindFrontier extends SerialTask {

    //Returns the frontier pheromone value of the edge
    private int getEdgeValue(LocalizationTriangle.Edge e){
        return controller.ownPublicVariables.getFrontierPheromone(controller, e.firstRobot, e.secondRobot);
    }

    //Returns an edge with minimal frontier pheromone value of the current triangle. If ambiguous, a random edge of the candidates is returned
    private LocalizationTriangle.Edge getMinEdge(){
        // Calculate the frontier pheromone values of the edges and the minimum of them
        int ab = getEdgeValue(controller.movementController.getLocalizationTriangle().getEdgeAB());
        int bc = getEdgeValue(controller.movementController.getLocalizationTriangle().getEdgeBC());
        int ca = getEdgeValue(controller.movementController.getLocalizationTriangle().getEdgeCA());
        int min = Math.min(ab, Math.min(bc, ca));

        //Set with all candidates (edges with minimal value)
        ArrayList<LocalizationTriangle.Edge> minEdges = new ArrayList<LocalizationTriangle.Edge>(3);
        if(ab == min) minEdges.add(controller.movementController.getLocalizationTriangle().getEdgeAB());
        if(bc == min) minEdges.add(controller.movementController.getLocalizationTriangle().getEdgeBC());
        if(ca == min) minEdges.add(controller.movementController.getLocalizationTriangle().getEdgeCA());

        // Return random edge of the candidates (never empty)
        return minEdges.get(new Random().nextInt(minEdges.size()));
    }

    //Check if the edge is a frontier edge
    private boolean isFrontier(LocalizationTriangle.Edge e){
        return getEdgeValue(e) == 0;
    }



    /**
     * The logic of the procedure
     */
    @Override
    public void serial_run() {

        /**
         * With this background loop we ensure properties that have to be valid all the time. If some property is violated, do some countermeasure.
         * The background loop runs parallel to the actual algorithm
         */
        addBackgroundLoop(new BackgroundLoop() {
            @Override
            void loop(RobotInterface robot, DOTRobotController controller) {
                //Check if overlapping triangle is detected. If one is detected, delete all its robots from the triangulation.
                ArrayList<LocalizationTriangle> enclosingTriangles = LocalizationTriangle.findTriangles(controller, 0.05f);
                if(enclosingTriangles.size()>1){
                    //TODO: Not that fast -> make sure it is not just an error
                    robot.send(new DeletionDemand(), enclosingTriangles.get(new Random().nextInt(enclosingTriangles.size())).robotA);
                    robot.send(new DeletionDemand(), enclosingTriangles.get(new Random().nextInt(enclosingTriangles.size())).robotB);
                    robot.send(new DeletionDemand(), enclosingTriangles.get(new Random().nextInt(enclosingTriangles.size())).robotC);
                    FINISH();
                    return;
                }

                //Check for competing robots
                LocalizationTriangle currentTriangle = controller.movementController.getLocalizationTriangle();
                if(currentTriangle !=null) {
                    PublicVariables.Triangle ownTriangle = new PublicVariables.Triangle(currentTriangle.robotA, currentTriangle.robotB, currentTriangle.robotC);
                    for (Neighbor tps : controller.neighborManager.getNeighbors()) {
                        if(tps.publicVariables.in_triangle!=null && tps.publicVariables.in_triangle.equals(ownTriangle)){
                            //There is an other robot in the triangle -> evasion
                            controller.addTask(new Evasion());
                            FINISH();
                            return;
                        }
                    }
                }

                //Sometimes new triangles have emerged but the controller is still fixed on some distanced triangle. Restart if this happens.
                // But be careful -> during the swap the closest triangle is not the current triangle
                if(currentTriangle!=null) {
                    LocalizationTriangle closestTriangle = LocalizationTriangle.getClosestTriangle(controller);
                    if(closestTriangle==null) { System.err.println(robot.getID()+": No closest triangle -> restart"); FINISH(); return;}
                    if (!closestTriangle.equals(currentTriangle)) {
                        if(Math.abs(closestTriangle.fuzzyContains(controller)-currentTriangle.fuzzyContains(controller))>0.3f){
                            System.err.println(robot.getID()+" closest triangle has a high difference to actual triangle -> restart "+closestTriangle.fuzzyContains(controller)+" "+currentTriangle.fuzzyContains(controller));
                            FINISH();
                            return;
                        }
                    }
                }

                if(currentTriangle==null){
                    controller.addTask(new FindTriangleTask());
                    FINISH();
                    return;
                }
            }
        });

        /**
         * The actual algorithm
         */
        try {
            //Get a first triangle
            controller.movementController.changeLocalizationTriangle(LocalizationTriangle.getClosestTriangle(controller));
            if(controller.movementController.getLocalizationTriangle()==null){
                controller.addTask(new FindTriangleTask());
                FINISH();
                return;
            }


            //If outside triangle, move onto the center of its closest edge
            if (!controller.movementController.getLocalizationTriangle().contains(controller)) {
                LocalizationTriangle.Edge closestEdge = controller.movementController.getLocalizationTriangle().getClosestEdge(controller);
                MOVE_TO(closestEdge.getCenter());
            }

            //Move Triangle.Center -> BestEdge.Center -> AdjacentTriangle.Center until the best edge is frontier
            while (true) {
                MOVE_TO(controller.movementController.getLocalizationTriangle().getCenter());
                LocalizationTriangle.Edge minEdge = getMinEdge();
                while(!isFrontier(minEdge) && controller.ownPublicVariables.getEvasionPheromone(controller, minEdge.firstRobot, minEdge.secondRobot)!=0){
                    controller.movementController.stopMoving();
                    UPDATE();
                    minEdge = getMinEdge();
                }
                if (isFrontier(minEdge)) {
                    MOVE_TO(minEdge.getCenter());
                    controller.addTask(new EdgeExpansion(robot, controller, controller.movementController, controller.movementController.getLocalizationTriangle(), minEdge));
                    FINISH();
                    return;
                } else {
                    MOVE_TO(minEdge.getCenter());
                    LocalizationTriangle goal_triangle = controller.movementController.getLocalizationTriangle().getAdjacentTriangle(controller, minEdge);
                    if(goal_triangle==null){
                        System.err.println("Robot-"+robot.getID()+" failed to get adjacent triangle, restart");
                        FINISH();
                        return;
                    }
                    controller.movementController.changeLocalizationTriangle(goal_triangle);
                }
                UPDATE();
            }


            /* *****************************************************************************
             * SOMETHING WENT WRONG
             ****************************************************************************** */
        } catch (CollisionException collision){
            //Collisions shouldn't happen. If it does, solve collision and restart.
            controller.addTask(new SolveCollisionTask());
            FINISH();
            return;
        } catch (TaskAbortedException tfe) {
            FINISH();
            return;
        }catch(TriangleMovementController.MovementControllerException mce){
            System.err.println(robot.getID()+": MovementController Failed");
            FINISH();
            return;
        } catch (Exception e){
            //Sometimes some data is not available for some reason. This can lead to null-pointer.
            //Doing special routines for these cases would drastically increase the code complexity. Thus we simply abort and restart.
            e.printStackTrace();
            FINISH();
            return;
        }
    }
}
