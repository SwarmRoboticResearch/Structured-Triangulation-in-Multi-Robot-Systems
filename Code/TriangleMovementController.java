package projects.triangulation;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import platypus3000.simulation.control.RobotInterface;
import platypus3000.utils.AngleUtils;
import projects.triangulation.Overlays.TrianglePointMovementOverlay;

/**
 * This class provides a controlled movement via reference triangles. In especially a goal position can be passed.
 * All movements in the work of this project thesis are performed by this controller.
 *
 * @author Dominik Krupke
 * Last modified: 12. Feb. 2016
 */
public class TriangleMovementController {

    // Configuration
    static final float VELOCITY=0.2f; //This is the value passed to the physic engine. Control moved distance by time
    static final float ROTATION_VELOCITY=1f; //This is the value passed to the physic engine. Control rotation by rotation time
    static final float ROTATE_IF_ANGLE_ABOVE = 0.3f; //If the goal direction differs by this angle from the assumed heading, a rotation is induced.
    static final float RELIABLE_MOVEMENT_LENGTH = 0.05f; //The distance need to be made to accept the new position and recalculate
    static final float GOAL_TOLERANCE = 0.07f; //The distance from which the position is allowed to differ from the goal
    static final int MIN_STRAIGHT_MOVEMENT_STEPS_AFTER_ROTATION = 10; // Forced straight movement steps after rotation (independent of moved distance)
    static final float ROTATION_VELOCITY_ESTIMATION_ADAPTION_RATE = 0.1f; // New = (1-x)*old + x*last_estimation
    static final float INITIAL_ROTATION_VELOCITY_ESTIMATION = 0.01f; //Initial value, should be not too large
    static final float ROTATION_VELOCITY_ESTIMATION_MAX = 5*INITIAL_ROTATION_VELOCITY_ESTIMATION; // Lower bound for the estimation
    static final float ROTATION_VELOCITY_ESTIMATION_MIN = 0.1f*INITIAL_ROTATION_VELOCITY_ESTIMATION; // Upper bound for the estimation

    LocalizationTriangle localizationTriangle; //The localization triangles that provides the coordinate system (and localization). Can be swapped.

    public Vec2 goal = null; //The goal, if set to null, do not move
    public Vec2 currentDirection = null; //The direction with respect to the the current localization triangle
    Vec2 lastPosition = null; //The last measured position

    float rotation_velocity_estimation = INITIAL_ROTATION_VELOCITY_ESTIMATION; //The initial value for the rotation velocity. Is adapted over time from measurements
    float rotation_time = 0; //Programmed rotation time (also needed to calculate the velocity afterwards)
    float already_executed_rotation_time = 0; //Executing the rotation since
    boolean ccw_rotation = true; //The direction to rotate in

    TrianglePointMovementOverlay overlay; //Visualization

    final DOTRobotController controller; //The controller that uses this TriangleMovementController-object.
    TriangleMovementController(DOTRobotController controller){
        this.controller = controller;
        overlay = new TrianglePointMovementOverlay(controller, "Movement and Localtion", this);
    }

    public Vec2 getLastPosition() throws MovementControllerException {
        if(localizationTriangle==null) throw new MovementControllerException("No localization triangle, thus no coordinate system");
        return (lastPosition!=null?lastPosition.clone():null);
    }

    public Vec2 getCurrentDirection() throws MovementControllerException {
        if(localizationTriangle==null) throw new MovementControllerException("No localization triangle, thus no coordinate system");
        return (currentDirection!=null?currentDirection.clone():null);
    }

    public Vec2 locate(DOTRobotController controller){
        if(localizationTriangle==null) return null;
        return localizationTriangle.locate(controller, lastPosition);
    }

    public LocalizationTriangle getLocalizationTriangle(){
        return localizationTriangle;
    }

    public void move_into_triangle(DOTRobotController controller) throws TriangleMovementController.MovementControllerException {
        if(!localizationTriangle.contains(controller)){
            LocalizationTriangle.Edge closestEdge = localizationTriangle.getClosestEdge(controller);
            controller.movementController.setGoalPosition(closestEdge.getCenter());
            if(controller.movementController.reachedGoal()){
                controller.movementController.setGoalPosition(localizationTriangle.getCenter());
            }
        } else {
            controller.movementController.setGoalPosition(localizationTriangle.getCenter());
        }
    }

    /**
     * Changes the LocalizationTriangle used for the coordinate system and localizations.
     * Can fail, which leads to a reset of the movement controller.
     * @param lt The new localization triangle
     */
   public void changeLocalizationTriangle(LocalizationTriangle lt) {
       if(localizationTriangle!=null && lt!=null && localizationTriangle.equals(lt)) return; //No change
       if(lt==null){
           goal = null;
           currentDirection = null;
           lastPosition = null;
           localizationTriangle = null;
           return;
       }
       if(localizationTriangle!=null) {
           if (goal != null) goal = lt.transformFrom(localizationTriangle, goal, controller);
           if(goal == null) {
               goal = null;
               currentDirection = null;
               lastPosition = null;
           }
           if (currentDirection != null)
               currentDirection = lt.transformFrom(localizationTriangle, currentDirection, controller).sub(lt.transformFrom(localizationTriangle, new Vec2(0, 0), controller));
           if (lastPosition != null) lastPosition = lt.transformFrom(localizationTriangle, lastPosition, controller);
       }
       localizationTriangle = lt;
    }

    public boolean reachedGoal() throws MovementControllerException {
        if(localizationTriangle==null) throw new MovementControllerException("No localization triangle, thus no coordinate system");
        return goal!=null && lastPosition!=null && lastPosition.sub(goal).lengthSquared()<= GOAL_TOLERANCE * GOAL_TOLERANCE;
    }

    boolean moveForwards = false;
    public void moveForwards(){
        moveForwards = true;
    }

    Boolean rotateCCW = null;
    public void rotate(boolean ccw){
        rotateCCW = ccw;
    }


    public void loop_prologue(RobotInterface robot){
        //Ensure the bounds on the rotation_velocity_estimation
        rotation_velocity_estimation = MathUtils.min(rotation_velocity_estimation, ROTATION_VELOCITY_ESTIMATION_MAX);
        rotation_velocity_estimation = MathUtils.max(rotation_velocity_estimation, ROTATION_VELOCITY_ESTIMATION_MIN);

        if(localizationTriangle!=null && !localizationTriangle.update(controller)){ //Update the localization triangle and remove it if it fails
            System.err.println(robot.getID()+": loop_prologue - Removing Localization Triangle "+localizationTriangle.toString());
            localizationTriangle = null;
            goal = null;
            currentDirection = null;
        }
    }

    int move_forwards_for =0;
    public void loop_epilogue(RobotInterface robot){
        if(!moveForwards && rotateCCW==null && goal == null) { //If no movement in programmed -> stop
            robot.setRotationVelocity(0);
            robot.setSpeed(0);
            return;
        }
        //If there is a manual movement command, execute it. It resets the current known direction.
        if(moveForwards || move_forwards_for>0) {
            robot.setSpeed(VELOCITY);
            robot.setRotationVelocity(0);
            moveForwards = false;
            //currentDirection = null;
            --move_forwards_for;
            return;
        }
        if(rotateCCW!=null){
            robot.setSpeed(0);
            robot.setRotationVelocity(rotateCCW?ROTATION_VELOCITY:-ROTATION_VELOCITY);
            rotateCCW = null;
            currentDirection = null;
            return;
        }

        if(localizationTriangle!=null && !localizationTriangle.update(controller)){
            System.err.println("Removing Localization Triangle "+localizationTriangle.toString());
            localizationTriangle = null;
            goal = null;
            currentDirection = null;
        }

        if(localizationTriangle==null){
            System.err.println("ROBOT-"+robot.getID()+" has a programmed movement but not localization triangle");
            goal = null;
            currentDirection = null;
            return; //Without localization triangle, we cannot do controlled movement
        }

        //If we do not know a last position, localize the robot
        if(lastPosition == null) lastPosition = locate(controller);

        if(goal!=null && lastPosition!=null && lastPosition.sub(goal).lengthSquared()> GOAL_TOLERANCE * GOAL_TOLERANCE){

            if(already_executed_rotation_time<rotation_time){ //executing rotation (takes multiple time steps)
                robot.setRotationVelocity((ccw_rotation?ROTATION_VELOCITY:-ROTATION_VELOCITY));
                robot.setSpeed(0);
                already_executed_rotation_time++;
                if(already_executed_rotation_time==rotation_time) move_forwards_for=MIN_STRAIGHT_MOVEMENT_STEPS_AFTER_ROTATION;
            } else {
                //Move until minimum movement length is reached
                Vec2 newPos = locate(controller);
                if(newPos == null) {
                    System.err.println("No localization -> no movement");
                    localizationTriangle = null;
                    return;
                } else if (newPos.sub(lastPosition).lengthSquared() < RELIABLE_MOVEMENT_LENGTH * RELIABLE_MOVEMENT_LENGTH) {
                    robot.setSpeed(VELOCITY);
                    robot.setRotationVelocity(0);
                } else {
                    robot.setSpeed(0);
                    robot.setRotationVelocity(0);

                    //Set new direction and estimate rotation velocity
                    Vec2 newDirection = newPos.sub(lastPosition);
                    if(currentDirection!=null) { //the current direction might not be known
                        float rotated = Math.abs(AngleUtils.normalizeToMinusPi_Pi(AngleUtils.getClockwiseRadian(newDirection, currentDirection)));
                        if (rotation_time > 0) {
                            rotation_velocity_estimation = (1-ROTATION_VELOCITY_ESTIMATION_ADAPTION_RATE) * rotation_velocity_estimation + ROTATION_VELOCITY_ESTIMATION_ADAPTION_RATE * (rotated / rotation_time);
                            rotation_time = 0;
                            //System.out.println(rotation_velocity_estimation);
                        }
                    }
                    currentDirection = newDirection;
                    lastPosition = newPos;

                    //Estimate angle correction and program it
                    float correctionAngle_ccw = -AngleUtils.getClockwiseRadian(currentDirection, goal.sub(lastPosition));
                    float smallestAngle = Math.abs(AngleUtils.normalizeToMinusPi_Pi(correctionAngle_ccw));
                    if (smallestAngle > ROTATE_IF_ANGLE_ABOVE) {
                        rotation_time = Math.round(smallestAngle/rotation_velocity_estimation);
                        ccw_rotation = ( AngleUtils.normalizeToMinusPi_Pi(correctionAngle_ccw) > 0 );
                        already_executed_rotation_time = 0;
                    }
                }
            }


        }
    }

    public void setGoalPosition(Vec2 p) throws MovementControllerException {
        assert p!=null;
        if(localizationTriangle==null) throw new MovementControllerException("No localization triangle, thus no coordinate system");
        this.goal = p;
    }

    public void resetGoal(){
        goal = null;
    }

    public void stopMoving(){
        resetGoal();
        rotateCCW = null;
        moveForwards = false;
    }

    public class MovementControllerException extends Exception {
        public MovementControllerException(String msg){
            super(msg);
        }
    }
}

