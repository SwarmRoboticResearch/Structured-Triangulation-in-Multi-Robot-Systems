package projects.triangulation.Overlays;

import org.jbox2d.common.Vec2;
import platypus3000.analyticstools.DynamicColor;
import platypus3000.analyticstools.LocalOverlay;
import platypus3000.simulation.ColorInterface;
import platypus3000.simulation.control.RobotController;
import platypus3000.simulation.control.RobotInterface;
import processing.core.PConstants;
import processing.core.PGraphics;
import projects.triangulation.NeighborManager.PublicVariables;

import java.util.Map;

/**
 * Created by doms on 2/1/16.
 */
public class EvasionPheromoneOverlay extends LocalOverlay {
    RobotInterface robot;
    PublicVariables ownState;
    DynamicColor color;
    DynamicColor interior;

    /**
     * The constructor. It needs the controller as a key, as only those overlays are executed, that are connected with
     * an active controller (controllers can be changed). The name is used for the Table in the GUI and has to be unique.
     * as otherwise two overlays gets the same row, which can lead to interferences.
     *
     * @param controller The controller, that uses this overlay
     * @param name       A unique name, which is also used for the Table in the GUI to activate it and change the colors.
     */
    public EvasionPheromoneOverlay(RobotInterface robot, PublicVariables ownState, RobotController controller, String name) {
        super(controller, name);
        this.robot = robot;
        this.ownState = ownState;
        this.color = getColor("Frontier");
        this.interior = getColor("Interior");
    }

    @Override
    public void drawBackground(PGraphics pGraphicsBackground, ColorInterface robot) {

    }

    @Override
    protected void drawForeground(PGraphics pGraphicsForeground) {
        for(Map.Entry<Integer, Integer> p: ownState.evasion_pheromone.entrySet()){
            try {
                Vec2 pos = robot.getNeighborhood().getById(p.getKey()).getLocalPosition();
                if (p.getValue() == 0) {
                    pGraphicsForeground.stroke(color.getColor());
                    pGraphicsForeground.fill(color.getColor());
                    pGraphicsForeground.line(0, 0, pos.x, pos.y);
                } else {
                    pGraphicsForeground.stroke(interior.getColor());
                    pGraphicsForeground.fill(interior.getColor());
                    pGraphicsForeground.line(0, 0, pos.x, pos.y);
                }
                pGraphicsForeground.textSize(0.2f);
                pGraphicsForeground.textAlign(PConstants.CENTER);
                pGraphicsForeground.text(p.getValue(), pos.x/2, pos.y/2);
            } catch(NullPointerException npe){

            }
        }
    }
}