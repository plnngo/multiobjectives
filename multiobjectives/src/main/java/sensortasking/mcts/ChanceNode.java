package sensortasking.mcts;

import lombok.Getter;

@Getter
public class ChanceNode extends Node{

    /** Micro action realised as pointing direction. */
    AngularDirection micro;

    /** Macro Action realised as tasking objective. */
    MacroAction macro;

    /** Execution duration. */
    double executionDuration;

    public ChanceNode(double duration, double utility, int numVisits, MacroAction objective, 
    AngularDirection pointing, Node parent) {

        parent.setChild(this);
        this.micro = pointing;
        this.macro = objective;
        super.numVisits = numVisits;
        super.utility = utility;
        super.parent = parent;
        this.executionDuration = duration;
        super.setEpoch(parent.getEpoch());
    }
}
