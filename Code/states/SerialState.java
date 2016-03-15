package projects.triangulation.states;

import platypus3000.simulation.control.RobotInterface;
import projects.triangulation.DOTRobotController;

/**
 * Created by doms on 1/26/16.
 */
public abstract class SerialState extends Thread implements State{
    RobotInterface robot = null;
    DOTRobotController controller = null;
    boolean isRunning = false;

    synchronized public void loop(RobotInterface robot, DOTRobotController controller){
        this.robot = robot;
        this.controller = controller;
        if(!isRunning){
            isRunning = true;
            run();
        } else {
            this.notify();
        }

        try {
            this.wait();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public synchronized void UPDATE(){
        try {
            this.notify();
            this.wait();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void run(){
        serial_run();
    }

    abstract public void serial_run();
}

