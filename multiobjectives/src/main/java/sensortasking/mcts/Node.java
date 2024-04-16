package sensortasking.mcts;

import java.util.List;

import lombok.Getter;

@Getter
public class Node {

    /** Parent node. */
    Node parent;

    /** Child nodes. */
    List<Node> children;

    /** Number of visits. */
    int numVisits;

    /** Utility value. */
    double utility;
}
