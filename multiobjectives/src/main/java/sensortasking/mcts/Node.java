package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

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
    private AbsoluteDate epoch;

    public void setChild(Node child) {
        children.add(child);
        child.parent = this;
    }

/*     public void setAsParentOf(Node child) {
        child.parent = this;
    } */

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

    public void setNumVisits(int numVisits) {
        this.numVisits = numVisits;
    }
}
