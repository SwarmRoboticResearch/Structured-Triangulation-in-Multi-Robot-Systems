package projects.triangulation.Environment;

import org.jbox2d.common.Vec2;
import platypus3000.simulation.Configuration;
import platypus3000.simulation.Robot;
import platypus3000.simulation.SimulationRunner;
import platypus3000.simulation.Simulator;
import platypus3000.visualisation.SwarmVisualisation;
import projects.triangulation.DOTRobotController;
import projects.triangulation.NeighborManager.NeighborManager;
import projects.triangulation.NeighborManager.PublicVariables;

import java.io.IOException;
import java.util.Stack;

/**
 * This is the base class for experiments. It runs a headless simulation and outputs a screenshot and statistics at the end.
 */
public class HeadlessRunner {

    private static void run_headless(String conf, String name, String env){
        try {
            Simulator sim = new Simulator(new Configuration(conf));

            SimulationRunner runner = new SimulationRunner(sim);

            //Entrance Triangle
            Robot portRobotA = sim.createRobot(-0.5f, 0, 0);
            Robot portRobotB = sim.createRobot(0, -0.86f);
            Robot portRobotC = sim.createRobot(0.5f, 0, 0);
            portRobotA.setController(new DOTRobotController(portRobotA.getID(), new PublicVariables.Triangle(portRobotA.getID(), portRobotB.getID(), portRobotC.getID()), portRobotC.getID(), portRobotA.getID()));
            portRobotB.setController(new DOTRobotController(portRobotB.getID(), new PublicVariables.Triangle(portRobotA.getID(), portRobotB.getID(), portRobotC.getID()), portRobotC.getID(), portRobotA.getID()));
            portRobotC.setController(new DOTRobotController(portRobotC.getID(), new PublicVariables.Triangle(portRobotA.getID(), portRobotB.getID(), portRobotC.getID()), portRobotC.getID(), portRobotA.getID()));

            //Add the triangulation overlay to the screenshot
            sim.configuration.overlayManager.getSharedProperties("RobotTriangulation").show_all = true;

            //Load the environment
            try {
                System.out.println("#Environment:");
                System.out.println(env);
                sim.environment.buildFromFile(env);
            } catch (Exception e) {
                e.printStackTrace();
            }

            //Output the neighborhood update probability
            System.out.println("#Neighborhood Update Probability");
            System.out.println("1/"+ NeighborManager.INVERSE_BROADCAST_PROBABILITY);

            Robot lastCreated = null;
            while (true) {
                if(sim.getTime()>500000){ //Abortion if it takes too long
                    System.out.println("ABORT. TOO SLOW!");
                    //Print
                    SwarmVisualisation.drawSwarmToPDF(name + "-abort", sim);
                    //Exit
                    System.exit(0);
                }
                runner.loop(50); //Execute 50 simulation steps

                //In case the entrance robots had a collision -> put to origin place.
                portRobotA.setGlobalPosition(-0.5f, 0);
                portRobotB.setGlobalPosition(0, -0.86f);
                portRobotC.setGlobalPosition(0.5f, 0);

                //Add new robot
                if(lastCreated == null || lastCreated.getController() == null || lastCreated.getGlobalPosition().sub(new Vec2(0.5f,-0.5f)).lengthSquared()>0.8f){
                    lastCreated = sim.createRobot(0f, -0.5f);
                    lastCreated.setController(new DOTRobotController());
                }
                //Check if any frontier edges left
                boolean done = true;
                for (Robot r : sim.getRobots()) {
                    DOTRobotController controller = (DOTRobotController) r.getController();
                    if (controller.ownPublicVariables.isStatic) {
                        //No frontier edges left
                        if (controller.ownPublicVariables.manages_a_frontier_edge()) {
                            done = false;
                            break;
                        }

                        //If in partition mode -> check that all robots are margin and no further robot will be removed
                        if(DOTRobotController.PARTITION_MODE && controller.ownPublicVariables.partition_root!=null && !controller.ownPublicVariables.partition_margin){
                            done = false;
                            break;
                        }
                    }
                }
                if (done) {
                    //Stats
                    print_stats(sim);
                    //Remove free robots
                    Stack<Robot> free_robots = new Stack<Robot>();
                    for (Robot r : sim.getRobots()) {
                        DOTRobotController controller = (DOTRobotController) r.getController();
                        if (!controller.ownPublicVariables.isStatic) {
                            free_robots.push(r);
                        }
                    }
                    for (Robot r : free_robots) {
                        sim.destroy(r);
                    }
                    runner.loop();
                    //Print
                    SwarmVisualisation.drawSwarmToPDF(name + "-final", sim);
                    //Exit
                    System.exit(0);
                }
            }
        } catch (Exception e){
            e.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * Printing the statistic of the simulation
     * @param sim Simulator of the simulation
     */
    private static void print_stats(Simulator sim){
        System.out.println("#Steps");
        System.out.println(sim.getTime());
        int robot_count = 0;
        for (Robot r : sim.getRobots()) {
            DOTRobotController controller = (DOTRobotController) r.getController();
            if (controller.ownPublicVariables.isStatic) {
                robot_count += 1;
            }
        }
        System.out.println("#Robots");
        System.out.println(robot_count);

        System.out.println("#Triangles");
        float area_sum = 0;
        for (Robot r : sim.getRobots()) {
            DOTRobotController controller = (DOTRobotController) r.getController();
            if (controller.ownPublicVariables.isStatic) {

                for(PublicVariables.Triangle t: controller.ownPublicVariables.triangles){
                    if(t.robotA==controller.getID()){
                        float a = sim.getRobot(t.robotA).getGlobalPosition().sub(sim.getRobot(t.robotB).getGlobalPosition()).length();
                        float b = sim.getRobot(t.robotB).getGlobalPosition().sub(sim.getRobot(t.robotC).getGlobalPosition()).length();
                        float c = sim.getRobot(t.robotC).getGlobalPosition().sub(sim.getRobot(t.robotA).getGlobalPosition()).length();
                        double area = 0.25f*Math.sqrt((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
                        area_sum+=area;
                        System.out.println("("+sim.getRobot(t.robotA).getGlobalPosition()+", "+sim.getRobot(t.robotB).getGlobalPosition()+", "+sim.getRobot(t.robotC).getGlobalPosition()+"), "+area);
                    }
                }
            }
        }
        System.out.println("#Area Sum");
        System.out.println(area_sum);

        System.out.println("#Wall Edges");
        for (Robot r : sim.getRobots()) {
            DOTRobotController controller = (DOTRobotController) r.getController();
            if (controller.ownPublicVariables.isStatic) {
                for(Integer n: controller.ownPublicVariables.wall_edges){
                    System.out.println("("+sim.getRobot(controller.getID()).getGlobalPosition()+", "+sim.getRobot(n).getGlobalPosition()+")");
                }
            }
        }

        System.out.println("#Blacklist Edges");
        for (Robot r : sim.getRobots()) {
            DOTRobotController controller = (DOTRobotController) r.getController();
            if (controller.ownPublicVariables.isStatic) {
                for(Integer n: controller.blacklist){
                    System.out.println("("+sim.getRobot(controller.getID()).getGlobalPosition()+", "+sim.getRobot(n).getGlobalPosition()+")");
                }
            }
        }


    }


    public static void main(String[] args) throws IOException {
        String conf = "/home/doms/Research/SwarmRoboticResearch/platypus3000/src/main/java/projects/triangulation/Environment/Environment1.properties";
        String name = "partition2";
        String env = "/home/doms/Research/SwarmRoboticResearch/platypus3000/src/main/java/projects/triangulation/Environment/Environment1-lineenvdata";
        //run_headless(args[0], args[1], args[2]);
        run_headless(conf, name, env);
    }
}
