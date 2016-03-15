package projects.triangulation.Messages;

import platypus3000.simulation.communication.MessagePayload;

/**
 * This is a simple message sent from a free robot that discovers an inconsistency in the triangulation to the responsible static robots.
 * The static robots receiving this message remove themselves from the triangulation and become free robots.
 * All information is provided by the message type (and possibly the sender's id).
 *
 * @author Dominik Krupke
 * Last modified: 10. Feb. 2016
 */
public class DeletionDemand implements MessagePayload{
    @Override
    public MessagePayload deepCopy() {
        return new DeletionDemand();
    }
}
