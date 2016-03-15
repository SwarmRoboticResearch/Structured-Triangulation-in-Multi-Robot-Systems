package projects.triangulation.Overlays;

import org.jbox2d.common.Vec2;
import platypus3000.analyticstools.DynamicColor;
import platypus3000.analyticstools.LocalOverlay;
import platypus3000.simulation.ColorInterface;
import platypus3000.simulation.control.RobotController;
import platypus3000.utils.AngleUtils;
import platypus3000.utils.VectorUtils;
import processing.core.PGraphics;
import projects.triangulation.LocalizationTriangle;
import projects.triangulation.TriangleMovementController;

/**
 * Created by doms on 1/26/16.
 */
public class TrianglePointMovementOverlay extends LocalOverlay {
    DynamicColor color_triangle;
    DynamicColor color_goal;

    TriangleMovementController tpm;

    /**
     * The constructor. It needs the controller as a key, as only those overlays are executed, that are connected with
     * an active controller (controllers can be changed). The name is used for the Table in the GUI and has to be unique.
     * as otherwise two overlays gets the same row, which can lead to interferences.
     *
     * @param controller The controller, that uses this overlay
     * @param name       A unique name, which is also used for the Table in the GUI to activate it and change the colors.
     */
    public TrianglePointMovementOverlay(RobotController controller, String name, TriangleMovementController tpm) {
        super(controller, name);
        this.tpm = tpm;
        color_triangle = getColor("Triangle");
        color_goal = getColor("Goal");
    }

    @Override
    public void drawBackground(PGraphics pGraphicsBackground, ColorInterface robot) {

    }

    @Override
    public void drawForeground(PGraphics pGraphicsForeground) {
        Vec2 A, B, C;
        Vec2 goal;
        LocalizationTriangle t = tpm.getLocalizationTriangle();
        try {
            if(t == null || tpm.getLastPosition() == null || tpm.getCurrentDirection()==null || tpm.currentDirection.lengthSquared()<0.001){
                A=null; B=null; C=null;
                return;
            }

        float counterclockwiseangle = AngleUtils.getRadian(tpm.getCurrentDirection());
        A = VectorUtils.transform(t.pos_A, tpm.getLastPosition(), -counterclockwiseangle);
        B = VectorUtils.transform(t.pos_B, tpm.getLastPosition(), -counterclockwiseangle);
        C = VectorUtils.transform(t.pos_C, tpm.getLastPosition(), -counterclockwiseangle);
        goal = tpm.goal;
        if(goal!=null){
            goal = VectorUtils.transform(goal, tpm.getLastPosition(), -counterclockwiseangle);
        } else {
            return;
        }

        if(A == null) return;
            pGraphicsForeground.stroke(color_triangle.getColor());
            pGraphicsForeground.fill(color_triangle.getColor());
            pGraphicsForeground.line(A.x, A.y, B.x, B.y);
            pGraphicsForeground.line(B.x, B.y, C.x, C.y);
            pGraphicsForeground.line(C.x, C.y, A.x, A.y);

        if(goal!=null){
            pGraphicsForeground.stroke(color_goal.getColor());
            pGraphicsForeground.fill(color_goal.getColor());
            pGraphicsForeground.line(0,0, goal.x, goal.y);
        }
        } catch (TriangleMovementController.MovementControllerException e) {

        }

    }
}
