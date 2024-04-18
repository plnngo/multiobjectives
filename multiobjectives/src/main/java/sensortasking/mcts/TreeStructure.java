package sensortasking.mcts;

import lombok.Getter;

@Getter
public class TreeStructure {

    /** Root node. */
    Node root;


    public int distanceFromRoot(Node descendant) {

        int distance = 0;
        Node ancestor = descendant.getParent(); 

        // Travel up the tree until root is reached
        while(!ancestor.equals(this.root)) {
            distance ++;
            ancestor = descendant.getParent(); 
        }

        return distance;
    }
    
}
