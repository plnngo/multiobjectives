package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.hipparchus.util.FastMath;
import org.orekit.time.AbsoluteDate;

public class MultiObjectiveMcts {

    /** Structure of the decision tree. */
    TreeStructure descisionTree;

    /** Root node. */
    Node initial;

    /** Objectives. */
    final List<MacroAction> objectives;

    /** Start date.*/
    final AbsoluteDate startCampaign;

    /** End date. */
    final AbsoluteDate endCampaign;

    /** Tuning parameter fur UCB */
    final double C = 2;

    /** Basic constructor.
     * 
     * @param descisionTree
     * @param objectives
     * @param start
     * @param end
     */
    public MultiObjectiveMcts(TreeStructure descisionTree, List<MacroAction> objectives,
                              AbsoluteDate start, AbsoluteDate end) {

        this.descisionTree = descisionTree;
        this.initial = this.descisionTree.getRoot();
        this.objectives = objectives;
        this.startCampaign = start;
        this.endCampaign = end;

    }

    public List<Node> select() {

        // Declare output
        List<Node> episode = new ArrayList<Node>();

        Node current = this.descisionTree.getRoot();
        List<Node> children = current.getChildren();
        episode.add(current);
        while(!children.isEmpty() || !Objects.isNull(children)) {

            // travers the tree until a leaf node is reached
            current = selectChild(current);
            children = current.getChildren();
            episode.add(current);
        }
        return episode;
    }

    protected Node selectChild(Node current) {
        double maxUct = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        double nP = current.getNumVisits();

        for (Node child : current.getChildren()){
            // Compute UCB 
            double v = child.getUtility();
            double n = child.getNumVisits();
            double ucb = v + C * FastMath.sqrt(FastMath.log(nP)/n);

            // search for child that maximises UCB
            if (ucb>maxUct) {
                potentiallySelected = child;
            }
        }
        return potentiallySelected;
    }
    
}
