package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.orekit.time.AbsoluteDate;

import lombok.Getter;

@Getter
public class Node {

    /** Parent node. */
    protected Node parent;

    /** Child nodes. */
    protected List<Node> children = new ArrayList<Node>();

    /** Number of visits. */
    protected int numVisits = 1;

    /** Utility value. */
    protected double utility;

    /** Reference epoch. */
    protected AbsoluteDate epoch;

    /** Node identifier. */
    protected long id;

    /** Accumulated utility vector. */
    double[] utilityVec;

    public void setChild(Node child) {
        children.add(child);
        child.parent = this;
    }

    public void removeChild(Node child) {
        children.remove(child);
    }

    public static void setParent(Node child, Node parent) {
        child.parent = parent;
        parent.children.add(child);
    }

    public void incrementNumVisits(){
        this.numVisits++;
    }

    public void setUtility(double value){
        this.utility = value;
    }

    public void setUtilityVec(double[] utility) {
        if (Objects.isNull(this.utilityVec)) {
            this.utilityVec = new double[utility.length];
        }
        for(int i=0; i<utility.length; i++) {
            this.utilityVec[i] = utility[i];
        }
    }

    public void setNumVisits(int numVisits) {
        this.numVisits = numVisits;
    }

    public void setId(long id) {
        this.id = id;
    }

    public void setEpoch(AbsoluteDate epoch) {
        this.epoch = epoch;
    }

    public void clearChildren() {
        this.children = new ArrayList<Node>();
    }
}
