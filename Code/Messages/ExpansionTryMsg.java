package projects.triangulation.Messages;

import platypus3000.simulation.communication.MessagePayload;

/**
 * Created by doms on 2/1/16.
 */
public class ExpansionTryMsg implements MessagePayload {

    public int min;
    public int max;
    public ExpansionTryMsg(int a, int b){
        min = Math.min(a,b);
        max = Math.max(a,b);
    }

    @Override
    public MessagePayload deepCopy() {
        return new ExpansionTryMsg(min, max);
    }
}
