package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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

    /**
     * 
     * @return
     */
    public List<Node> select() {

        // Declare output
        List<Node> episode = new ArrayList<Node>();

        Node current = this.descisionTree.getRoot();
        episode.add(current);
        List<Node> children = current.getChildren();
        
        while(!children.isEmpty() || !Objects.isNull(children)) {

            // travers the tree until a leaf node is reached
            current = selectChild(current);
            children = current.getChildren();
            episode.add(current);
        }
        return episode;
    }

    public Node expand(List<Node> episode, Node leaf){

        // Sample a macro action
        MacroAction objective = null;
        ChanceNode microAction = objective.setMicroAction();

        AngularDirection pointing = microAction.getMicro();
        Map.Entry<Outcome, Double> outcomeReward = objective.sampleOutcome();
        double utility = objective.getUtility();
        Node nextChance = new ChanceNode(objective, microAction, leaf, outcomeReward);
        Node nextDecision = new DecisionNode(utility, outcomeReward);

        // Extend the tree by new decision and chance nodes
        nextDecision.setChild(nextChance);
        leaf.setChild(nextDecision);

        // Extend episode by new decision and chance nodes too
        episode.add(nextDecision);
        episode.add(nextChance);


        return leaf;
    }

    public List<Node> simulate(Node leaf) {

        // Declare output
        List<Node> episode = new ArrayList<Node>();
        episode.add(leaf);

        Node current = leaf;
        AbsoluteDate currentEpoch = leaf.getEpoch();

        while(currentEpoch.compareTo(this.endCampaign) <= 0) {
            current = expand(episode, current);
            episode.add(current);
            currentEpoch = current.getEpoch();
        }
        return episode;
    }

    /**
     * Update the state of every parent node along the episode from the initial node down to the 
     * newest expanded node.
     * 
     * @param selected          List of nodes that has been filled by calling {@link #select()}.
     * @param last              Newest expanded node.
     */
    public void backpropagate(List<Node> selected, Node last) {

        // Search for corresponding node in tree
        Node findCurrent = this.initial;
        for (int i=1; i<selected.size(); i++) {
            int ancestorIndex = findCurrent.getChildren().indexOf(selected.get(i));
            findCurrent = findCurrent.getChildren().get(ancestorIndex);

            // Update status of ancestor node
            findCurrent.incrementNumVisits();
            double updatedUtility = findCurrent.getUtility() + last.getUtility();
            findCurrent.setUtility(updatedUtility);
        }
    }

    /**
     * 
     * @param current
     * @return
     */
    protected Node selectChild(Node current) {
        double maxUcb = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        double nP = current.getNumVisits();

        for (Node child : current.getChildren()){
            // Compute UCB 
            double v = child.getUtility();
            double n = child.getNumVisits();
            double ucb = v + C * FastMath.sqrt(FastMath.log(nP)/n);

            // search for child that maximises UCB
            if (ucb>maxUcb) {
                potentiallySelected = child;
                maxUcb = ucb;
            }
        }
        return potentiallySelected;
    }

        
    public static void main(String[] args) {
        List<Node> test = new ArrayList<Node>();
        Node decision = new Node();
        decision.setUtility(9);
        Node chance = new Node();
        decision.setChild(chance);
        chance.setUtility(7);
        test.add(decision);
        test.add(chance);
        System.out.println("Utility of initial chance node: " + chance.getUtility());

        //Extract node
        Node extractedChance = test.get(1);
        extractedChance.setUtility(10);
        System.out.println("Utility of extracted chance node:" + extractedChance.getUtility());

        for(Node node : test) {
            System.out.println("Utilities in tree: " + node.getUtility());
        }
    }
}
