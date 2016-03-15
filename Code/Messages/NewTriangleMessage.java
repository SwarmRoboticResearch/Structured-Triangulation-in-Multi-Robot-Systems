package projects.triangulation.Messages;

import platypus3000.simulation.communication.MessagePayload;
import projects.triangulation.NeighborManager.PublicVariables;

/**
 * Created by doms on 1/10/16.
 */
public class NewTriangleMessage implements MessagePayload {
    public PublicVariables.Triangle triangle;

    public NewTriangleMessage(PublicVariables.Triangle triangle){
        this.triangle = triangle;
    }

    @Override
    public MessagePayload deepCopy() {
        NewTriangleMessage cloned = new NewTriangleMessage(triangle.clone());
        return cloned;
    }
}
