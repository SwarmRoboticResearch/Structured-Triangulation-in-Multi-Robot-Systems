package projects.triangulation.Environment;

import org.jbox2d.common.Vec2;
import platypus3000.simulation.Configuration;
import platypus3000.simulation.Robot;
import platypus3000.simulation.Simulator;
import platypus3000.utils.RobotCreator;
import platypus3000.visualisation.VisualisationWindow;
import projects.triangulation.DOTRobotController;
import projects.triangulation.NeighborManager.PublicVariables;

import java.awt.*;
import java.io.IOException;

/**
 * Created by doms on 1/3/16.
 */
public class Environment1 {
    Simulator sim;
    public Environment1() throws IOException {
        sim = new Simulator(new Configuration("/home/doms/Research/SwarmRoboticResearch/platypus3000/src/main/java/projects/triangulation/Environment/Environment1.properties"));

        Robot portRobotA = sim.createRobot(-.5f,0,0);
        Robot portRobotB = sim.createRobot(0, -0.87f);
        Robot portRobotC = sim.createRobot(0.5f,0,0);
        portRobotA.setController(new DOTRobotController(portRobotA.getID(), new PublicVariables.Triangle(portRobotA.getID(), portRobotB.getID(), portRobotC.getID()), portRobotC.getID(), portRobotA.getID()));
        portRobotB.setController(new DOTRobotController(portRobotB.getID(), new PublicVariables.Triangle(portRobotA.getID(), portRobotB.getID(), portRobotC.getID()), portRobotC.getID(), portRobotA.getID()));
        portRobotC.setController(new DOTRobotController(portRobotC.getID(), new PublicVariables.Triangle(portRobotA.getID(), portRobotB.getID(), portRobotC.getID()), portRobotC.getID(), portRobotA.getID()));
        //robot.setController(null);
        try {
            sim.environment.buildFromFile("/home/doms/Research/SwarmRoboticResearch/platypus3000/src/main/java/projects/triangulation/Environment/Environment1-lineenvdata");
        } catch (Exception e) {
            e.printStackTrace();
        }


        VisualisationWindow w = new VisualisationWindow(sim, new Dimension(1000,700));
        w.visualisation.setRobotCreator(new RobotCreator() {
            @Override
            public void createRobot(Simulator sim, int id, float x, float y, float angle) {
                Robot r = sim.createRobot(x, y);
                r.setController(new DOTRobotController());
            }
        });
        while(true){
            synchronized (sim) {
//                portRobotA.setGlobalPosition(0, 0);
//                portRobotB.setGlobalPosition(0.5f, -0.7f);
//                portRobotC.setGlobalPosition(1, 0);
                addNewRobot();


            }
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }



    Robot lastCreated = null;
    public void addNewRobot(){
        if(lastCreated == null || lastCreated.getController() == null || lastCreated.getGlobalPosition().sub(new Vec2(0.5f,-0.5f)).lengthSquared()>0.8f){
            lastCreated = sim.createRobot(0f, -0.5f);
            lastCreated.setController(new DOTRobotController());
        }
    }


    public static void main(String[] args) throws IOException {
        new Environment1();
    }
}
