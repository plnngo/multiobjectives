package sensortasking.mcts;

import java.util.List;

import lombok.Getter;

@Getter
public class Node {

    /** Parent node. */
    private Node parent;

    /** Child nodes. */
    private List<Node> children;

    /** Number of visits. */
    private int numVisits;

    /** Utility value. */
    private double utility;
}
