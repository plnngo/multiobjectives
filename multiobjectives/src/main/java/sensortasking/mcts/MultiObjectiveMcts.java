package sensortasking.mcts;

import java.util.List;

import org.orekit.time.AbsoluteDate;

public class MultiObjectiveMcts {

    /** Structure of the decision tree. */
    TreeStructure descisionTree;

    /** Root node. */
    Node initial;

    /** Objectives. */
    List<MacroAction> objectives;

    /** Start date.*/
    AbsoluteDate startCampaign;

    /** End date. */
    AbsoluteDate endCampaign;

    public List<Node> select() {
        return null;
    }
    
}
