package projects.triangulation.Messages;

import platypus3000.simulation.communication.MessagePayload;

/**
 * Created by doms on 1/29/16.
 */
public class DeletionNotification implements MessagePayload {
    @Override
    public MessagePayload deepCopy() {
        return new DeletionNotification();
    }
}
