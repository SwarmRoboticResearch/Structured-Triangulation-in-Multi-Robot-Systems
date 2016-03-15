package projects.triangulation.Messages;

import platypus3000.simulation.communication.MessagePayload;

/**
 * Created by doms on 1/10/16.
 */
public class WallMessage implements MessagePayload {
    public int robotA;
    public int robotB;
    public WallMessage(int robotA, int robotB){
        this.robotA = robotA; this.robotB = robotB;
    }

    @Override
    public MessagePayload deepCopy() {
        return new WallMessage(robotA, robotB);
    }
}
