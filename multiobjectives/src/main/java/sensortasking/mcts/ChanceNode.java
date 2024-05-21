package sensortasking.mcts;

import org.orekit.time.AbsoluteDate;

import lombok.Getter;

@Getter
public class ChanceNode extends Node{

    /** Micro action realised as pointing direction. */
    AngularDirection micro;

    /** Macro Action realised as tasking objective. */
    Objective macro;

    /** Execution duration. */
    private AbsoluteDate[] executionDuration;

    public ChanceNode(AbsoluteDate[] obsTimeInterval, double utility, int numVisits, Objective objective, 
    AngularDirection pointing, Node parent) {

        parent.setChild(this);
        this.micro = pointing;
        this.macro = objective;
        super.numVisits = numVisits;
        super.utility = utility;
        super.parent = parent;
        this.executionDuration = obsTimeInterval;
        super.setEpoch(parent.getEpoch());
    }
}
