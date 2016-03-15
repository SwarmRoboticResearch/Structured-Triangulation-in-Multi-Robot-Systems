package projects.triangulation.NeighborManager;

import platypus3000.simulation.communication.Message;
import platypus3000.simulation.communication.MessagePayload;
import platypus3000.simulation.control.RobotInterface;
import platypus3000.simulation.neighborhood.NeighborView;

import java.util.HashMap;
import java.util.Random;
import java.util.Stack;

/**
 * This class manages the neighbors of a robot. Public variables and the own sensing of the neighbor (in this case only distance) are combined.
 * @author Dominik Krupke
 * Last changed: 12. Feb 2016
 */
public class NeighborManager {
    final static int MAX_AGE = 30; //If a robot doesn't get updates from a non static neighbor for MAX_AGE time steps, it is assumed to be vanished
    final static int MAX_AGE_STATIC = 300; //If a robot doesn't get updates from a static neighbor for MAX_AGE_STATIC time steps, it is assumed to be vanished. The value is this
    // high as free robots can wait in the center of the connecting edge and in case of Line-Of-Sight-Constraint this would lead to a assumed disconnection.

    public final static int INVERSE_BROADCAST_PROBABILITY = 2; // A broadcast messsage is only sent with a probability of 1/X. Used to simulate usage with unreliable communication

    final PublicVariables ownPublicVariables; //The public variables of the robot that owns this manager

    HashMap<Integer, Neighbor> neighbors = new HashMap<Integer, Neighbor>(); //The entries of the neighbors

    int roundsUntilStabilized = 30; //Give some time to gather information

    /**
     * Creates the NeighborManager and sets the containter that contains the own public variables
     * @param ownPublicVariables The container with the own public variables
     */
    public NeighborManager(PublicVariables ownPublicVariables){
        this.ownPublicVariables = ownPublicVariables;
    }

    /**
     * To be executed at the beginning of a time step. It updates the neighbors' entries
     * @param robot The robot interfac to query the received messages
     */
    public void loop_prologue(RobotInterface robot){
        //Increase age and remember candidates for removal
        Stack<Neighbor> toRemove = new Stack<Neighbor>();
        for(Neighbor n: neighbors.values()){
            n.age+=1;
            if(n.age>(ownPublicVariables.isStatic&&n.publicVariables.isStatic?MAX_AGE_STATIC:MAX_AGE)) toRemove.add(n);
        }

        //Process messages
        for(Message m: robot.incomingMessages()){
            if(m.msg instanceof NeighborMessage){
                if(!robot.getNeighborhood().contains(m.sender)) {m.delete(); continue;}
                NeighborView nv = robot.getNeighborhood().getById(m.sender);
                if(!neighbors.containsKey(m.sender)) neighbors.put(m.sender, new Neighbor(m.sender));
                Neighbor n = neighbors.get(m.sender);
                n.publicVariables = ((NeighborMessage) m.msg).publicVariables;
                n.distance = nv.getDistance();
                n.age = 0;
                m.delete();
            }
        }

        //Remove outdated neighbors
        for(Neighbor n: toRemove){
            if(n.age > MAX_AGE) neighbors.remove(n.ID);
        }


    }

    /**
     * To be executed at the end of a time step to broadcast the latest values of the own public variables
     * @param robot The robot interface in order to send the messages
     */
    public void loop_epilogue(RobotInterface robot){
        if(INVERSE_BROADCAST_PROBABILITY==1 || new Random().nextInt(INVERSE_BROADCAST_PROBABILITY)==0) {
            robot.send(new NeighborMessage(ownPublicVariables.createMessageClone()));
        }
        if(roundsUntilStabilized>0) --roundsUntilStabilized;
    }

    /**
     * A newly add robot needs some time to gather neighborhood information.
     * Especially those that are initially static robots.
     * @return True if the neighborhood should be completely sensed.
     */
    public boolean hasStabilized(){
        return roundsUntilStabilized==0;
    }

    /**
     * Returns true if the neighbor manager has an entry for the robot with the given id
     * @param nbr The id of the neighbor
     * @return
     */
    public boolean contains(int nbr){
        return neighbors.containsKey(nbr);
    }

    /**
     * Returns the neighbor entry for the corresponding id
     * @param id The id of the neighbor
     * @return The entry of the neighbor or null if it is not available
     */
    public Neighbor getNeighbor(int id){
        return neighbors.get(id);
    }

    /**
     * All neighbors in an iterable form for usage in a for-loop
     * @return Iterable neighbor set
     */
    public Iterable<Neighbor> getNeighbors(){
        return neighbors.values();
    }

    /**
     * A container for broadcasting the own information/public variables.
     */
    private class NeighborMessage implements MessagePayload {
        NeighborMessage(PublicVariables publicVariables){
            this.publicVariables = publicVariables;
        }
        PublicVariables publicVariables;

        @Override
        public MessagePayload deepCopy() {
            return new NeighborMessage(publicVariables.createMessageClone());
        }
    }
}
