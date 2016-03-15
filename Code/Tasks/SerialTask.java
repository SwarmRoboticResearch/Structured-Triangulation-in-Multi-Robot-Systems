package projects.triangulation.Tasks;

import org.jbox2d.common.Vec2;
import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;
import projects.triangulation.TriangleMovementController;

import java.util.ArrayList;

/**
 * Created by doms on 1/27/16.
 */
public abstract class SerialTask extends Thread implements TaskState {

    RobotInterface robot = null;
    DOTRobotController controller = null;
    boolean isRunning = false;
    boolean finished = false;
    ArrayList<BackgroundLoop> backgroundLoops = new ArrayList<BackgroundLoop>();

    synchronized public boolean loop(RobotInterface robot, DOTRobotController controller){
        if(finished) return true;
        this.robot = robot;
        this.controller = controller;
        if(!isRunning){
            isRunning = true;
            start();
        } else {
            this.notify();
        }
        try {
            this.wait();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return finished;
    }

    public void MOVE_TO(Vec2 p) throws CollisionException, TaskAbortedException, TriangleMovementController.MovementControllerException{
        controller.movementController.setGoalPosition(p);
        while(!controller.movementController.reachedGoal()){
            if(robot.hasCollision()) throw new CollisionException();
            controller.movementController.setGoalPosition(p);
            UPDATE();
        }
    }

    public synchronized void UPDATE() throws TaskAbortedException {
        if(finished) throw new TaskAbortedException();
        for(BackgroundLoop d: backgroundLoops){
            d.loop(robot, controller);
        }
        if(finished) throw new TaskAbortedException();
        try {
            this.notify();
            this.wait();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public synchronized void FINISH(){
        this.notify();
        finished = true;
    }

    public void run(){
        serial_run();
        FINISH();
    }

    abstract public void serial_run();

    public void addBackgroundLoop(BackgroundLoop a){
        backgroundLoops.add(a);
    }

    abstract class BackgroundLoop {
        abstract void loop(RobotInterface robot, DOTRobotController controller);
    }

    class CollisionException extends Exception{

    }
    class TaskAbortedException extends Exception{}
}
