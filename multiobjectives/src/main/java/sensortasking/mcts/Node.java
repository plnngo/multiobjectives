package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import lombok.Getter;

@Getter
public class Node {

    /** Parent node. */
    private Node parent;

    /** Child nodes. */
    private List<Node> children = new ArrayList<Node>();

    /** Number of visits. */
    private int numVisits;

    /** Utility value. */
    private double utility;

    public void setChild(Node child) {
        children.add(child);
    }
}
