package projects.triangulation.NeighborManager;

/**
 * This is a neighbor entry. We do not use the original neighborhood system of the simulator as it is much too powerful and not adapted to our case.
 * In especially, this neighbor entry also contains the public variables thus a lot of double checking (does neighbor exist, do we have its public variables) can be dropped
 *
 * @author Dominik Krupke
 * Last modified: 12.02.2016
 */
public class Neighbor {
    public PublicVariables publicVariables; //The public variables of this neighbor, never null
    public float distance; //The distance to this neighbor
    int age = 0; //The age of the neighbor entry (last time we got a message from it) as we might not get a message every round
    public final int ID; //The id of the neighbor

    public Neighbor(int ID){
        this.ID = ID;
    }
}
