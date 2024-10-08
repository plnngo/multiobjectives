package sensortasking.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;
import sensortasking.stripescanning.Stripe;
import sensortasking.stripescanning.Tasking;
import tools.WeightedRandomNumberPicker;

@Getter
public class MultiObjectiveMcts {

    /** Structure of the decision tree. */
    //TreeStructure descisionTree;

    /** Root node. */
    Node initial;

    /** Objectives. */
    static List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));

    /** Start date.*/
    final AbsoluteDate startCampaign;

    /** End date. */
    final AbsoluteDate endCampaign;

    /** Tuning parameter fur UCB. */
    final static double C = 20.;

    /** Topocentric horizon frame. */
    final TopocentricFrame stationFrame;

    /** Earth centered Earth fixed frame. */
    final Frame j2000 = FramesFactory.getEME2000();

    /** Tuning parameter (0;1) for progressive widening */
    final static double alpha = 0.3;

    /** List of objects of interest to be tracked. */
    final List<ObservedObject> trackedObjects;

    /** List of objects that have been detected.*/
    List<ObservedObject> detectedObjects = new ArrayList<ObservedObject>();

    /** Stripe for searching objective. */
    final Stripe scanStripe; 

    /** Number of exposures within stripe scanning algorithm. */
    final int numExpo = 5;

    /** Observation station (TODO: implement sensor for tracking objective). */
    final Sensor sensor;

    /** Basic constructor.
     * 
     * @param descisionTree
     * @param objectives
     * @param start
     * @param end
     */
    public MultiObjectiveMcts(Node descisionTree, List<String> objectives,
                              AbsoluteDate start, AbsoluteDate end, TopocentricFrame stationFrame,
                              List<ObservedObject> trackedObjects, List<ObservedObject> detectedObjects,
                              Sensor sensor) {

        this.initial = descisionTree;
        MultiObjectiveMcts.objectives = objectives;
        this.startCampaign = start;
        this.endCampaign = end;
        this.stationFrame = stationFrame;
        this.trackedObjects = trackedObjects;
        this.detectedObjects = detectedObjects;
        this.sensor = sensor;

        //this.scanStripe = null;
        this.scanStripe = computeScanStripe();
    }

  
    public Stripe computeScanStripe() {

        Tasking survey = new Tasking(sensor, this.startCampaign, this.endCampaign, this.numExpo);
        Stripe[] stripes = survey.computeScanStripes();

        // TODO: select fixed stripe that does not enter earth shadow
        return stripes[1];
    }

    public List<Node> run(Node initial, int iterations) {

        List<Node> outputUCB = new ArrayList<Node>();
        List<Node> outputRobustMax = new ArrayList<Node>();

        for(int i=0; i<iterations; i++) {

                        
            selectNew(initial);
            // retrieve updated root node
            //System.out.println("Num of visits " + initial.getNumVisits());

            // Retrieve pointing strategy UCB
            Node current = initial;
            //initial.incrementNumVisits();
            outputUCB.add(initial);
            // Travers decision until leaf node
            while(!Objects.isNull(current) && current.getChildren().size() !=0){
                current = selectChildUCB(current);
                outputUCB.add(current);
/*                 if (!Objects.isNull(current)){
                    current.incrementNumVisits();
                } */
            }

            outputUCB = new ArrayList<Node>();
        }

        // Retrieve pointing strategy UCB
        Node current = initial;
/*         outputUCB.add(initial);
        // Travers decision until leaf node
        while(!Objects.isNull(current) && current.getChildren().size() !=0){
            current = selectChildUCB(current);
            outputUCB.add(current);
        } */

        //current = initial;
        outputRobustMax.add(initial);
        // Travers decision until leaf node
        while(!Objects.isNull(current) && current.getChildren().size() !=0){
            current = selectChildRobustMax(current);
            outputRobustMax.add(current);
        }
        return outputRobustMax;
    }


    /**
     * Given the initial node {@code root}, the next child is selected based on the Upper 
     * Confidence Bound criteria until the termination condition is reached given by 
     * {@link #endCampaign}.
     * 
     * @param current           Initial node.
     * @return                  Last node that is selected according to UCB criteria.
     */
    public Node select(Node current) {

        List<Node> children = current.getChildren();
        // check progressive widening condition
        if(current.getClass().getSimpleName().equals("DecisionNode")) {
            boolean expandable = true;
            while(children.size() <= FastMath.pow(current.getNumVisits(), alpha) && expandable) {

                // allow expansion of new node
                DecisionNode leaf = expand((DecisionNode) current, false);
                if (Objects.isNull(leaf)){
                    // objects not observable 
                    children = current.getChildren();
                    expandable = false;
                    continue;

                } else if (leaf.getEpoch().compareTo(endCampaign) >= 0) {
                    // already reached end of campaign
                    return current;
                }
                expandable = true;
                List<Node> simulated = simulate(leaf, endCampaign);
                if (simulated.size() != 0) {
                    backpropagate(leaf, simulated.get(simulated.size()-1));
                } else {
                    backpropagate(leaf, null);
                }
                children = current.getChildren();
            } 
        }

        if (current.getEpoch().compareTo(endCampaign) <= 0) {
            Node nextChild = selectChildUCB(current);

            if (Objects.isNull(nextChild)) {
                // no time for further tasks
                return current;
            }
            
            if(nextChild.getClass().getSimpleName().equals("ChanceNode")) {
                System.out.println("---");
                System.out.println("Selected node");
                ChanceNode chanceSelected = (ChanceNode) nextChild;
                if(chanceSelected.getMacro().getClass().getSimpleName().equals("TrackingObjective")) {
                    TrackingObjective macro = (TrackingObjective)chanceSelected.getMacro();
                    System.out.println(macro.getLastUpdated());
                    System.out.println(chanceSelected.getMicro().getDate());
                    System.out.println("RA: " + FastMath.toDegrees(chanceSelected.getMicro().getAngle1()));
                    System.out.println("DEC: " + FastMath.toDegrees(chanceSelected.getMicro().getAngle2()));
                    
                    System.out.println("Utility of root node " + this.initial.getUtility());
                } else {
                    System.out.println("Scanning stripe, first pointing direction:");
                    System.out.println("RA in EME2000: " + FastMath.toDegrees(chanceSelected.getMicro().getAngle1()));
                    System.out.println("DEC in EME2000: " + FastMath.toDegrees(chanceSelected.getMicro().getAngle2()));
                    System.out.println(chanceSelected.getEpoch());
                }
            }
            return select(nextChild);
        } else {
            return current;
        }
    }

    public Node selectNew(Node current) {

        boolean widening = progressiveWidening(current);
        if (!widening) {
            // already reached end of campaign
            return current;
        }

        //current.incrementNumVisits();
        Node nextChild = current;
        while(nextChild.getChildren().size() != 0) {
            nextChild = selectChildUCB(nextChild);
            if (Objects.isNull(nextChild)) {
                // no time for further tasks
                return current;
            }
            //nextChild.incrementNumVisits();
        }
        // Reached leaf node but not end of campaign yet --> progressiveWidening()
        widening = progressiveWidening(nextChild);
        /* if (!widening) {
            // already reached end of campaign --> TODO: need to backpropagate
            while(!nextChild.equals(this.initial)) {
                nextChild.incrementNumVisits();
                nextChild = nextChild.getParent();
            }
            this.initial.incrementNumVisits();
            return current;
        } */
        return current;
    }

    private boolean progressiveWidening(Node current) {
        if(current.getEpoch().compareTo(endCampaign) >= 0) {
            backpropagate(current, null);
            return false;
        }
        List<Node> children = current.getChildren();
        // check progressive widening condition
        if(current.getClass().getSimpleName().equals("DecisionNode")) {
            boolean expandable = true;
            while(children.size() <= FastMath.pow(current.getNumVisits(), alpha) && expandable) {

                // allow expansion of new node
                DecisionNode leaf = expand((DecisionNode) current, false);
                if (Objects.isNull(leaf)){
                    // objects not observable 
                    children = current.getChildren();
                    expandable = false;
                    continue;

                } else if (leaf.getEpoch().compareTo(endCampaign) >= 0) {
                    // already reached end of campaign
                    if (((ChanceNode)leaf.getParent()).getMacro().getClass().getSimpleName().equals("TrackingObjective")) {
                        leaf.getParent().getParent().clearChildren();
                        return false;
                    }
                }
                expandable = true;
                List<Node> simulated = simulate(leaf, endCampaign);
                if (simulated.size() != 0) {
                    backpropagate(leaf, simulated.get(simulated.size()-1));
                } else {
                    backpropagate(leaf, null);
                }
            } 
        }
        return true;
    }


    /**
     * Expand decision tree at the given leaf node by two further nodes, i.e. a new chance and a 
     * new decision node .
     * 
     * @param leaf              Current decision leaf node.
     * @param simulationPhase   True if we are in simulation phase, false if only a node is simply 
     *                          expanded.
     * @return                  Next decision leaf node that is added to the decision tree together
     *                          with its parent chance node. 
     */
    public DecisionNode expand(DecisionNode leaf, boolean simulationPhase){

        //Node leaf = selected.get(selected.size()-1);
        //String nodeType = leaf.getClass().getSimpleName();
        ChanceNode expandedChance = null;
        DecisionNode expandedDecision = null;
        
        // Expand by Chance node first
        // Need to sample a new pair of macro and micro action
        //DecisionNode castedLeaf = (DecisionNode) leaf;
        //double[] weights = leaf.getWeights();
        double[] weights = new double[]{0.5, 0.5};

        // Generate array filled with indexes representing the objective IDs
        int[] indexObjective = new int[weights.length];
        for (int i=0; i<weights.length; i++) {
            indexObjective[i] = i;
        }
        int indexSelectedObjective = 
            WeightedRandomNumberPicker.pickNumber(indexObjective, weights);
        Objective objective;
        //List<ObservedObject> propEnviroment = new ArrayList<ObservedObject>();
        switch (indexSelectedObjective) {
            case 0:
                boolean searchPossible = true;
                boolean searchAlreadyPerformed = false;
                // make sure that tree does not get expanded by the same node that already exist among siblings
                for(Node sibling : leaf.getChildren()) {
                    ChanceNode chance = (ChanceNode)sibling;
                    
                    if (chance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                        searchAlreadyPerformed = true; // Search is already performed by another sibling
                    }
                }
                // Time until end of observation campaign
                double leftT = this.endCampaign.durationFrom(leaf.getEpoch());
                if(leftT < scanStripe.getStripeT(numExpo)){
                    searchPossible = false; // Not enough time to complete search
                }
                if(searchPossible && !searchAlreadyPerformed) {
                    objective = new SearchObjective(stationFrame, scanStripe, numExpo, sensor);
                    break;
                } else if(!searchAlreadyPerformed && leaf.getTimeResources()[1] < TrackingObjective.allocation 
                                                                                    + this.sensor.getSettlingT() 
                                                                                    + TrackingObjective.preparation 
                                                                                    + this.sensor.getExposureT()) {
                    objective = new SearchObjective(stationFrame, scanStripe, numExpo, sensor);
                    break;
                } else {
                    indexSelectedObjective = 1;
                }

            case 1:
                // Macro action = track
                AbsoluteDate measEpoch = 
                    leaf.getEpoch().shiftedBy(TrackingObjective.allocation 
                                                + this.sensor.getSettlingT() 
                                                + TrackingObjective.preparation 
                                                + this.sensor.getExposureT()/2);

                // Extract new station position
                Transform horizonToEci = stationFrame.getTransformTo(j2000, measEpoch); 
                Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
                Transform eciToTopo = new Transform(measEpoch, coordinatesStationEci.negate());
                Frame topoInertial = new Frame(j2000, eciToTopo, "Topocentric", true);

                // make sure that tree does not get expanded by the same node that already exist among siblings
                DecisionNode current = leaf;
                while(current.getPropEnvironment().size()==0) {
                    current = ((DecisionNode)current.getParent().getParent());
                }
                List<ObservedObject> ooi = new ArrayList<>(current.getPropEnvironment());
                for(Node sibling : leaf.getChildren()) {
                    ChanceNode chance = (ChanceNode)sibling;
                    if (chance.getMacro().getClass().getSimpleName().equals("TrackingObjective")) {
                        TrackingObjective track = (TrackingObjective)chance.getMacro();
                        long idAlreadyTracked = track.getLastUpdated();
                        int index = -1;
                        for(int i=0; i<ooi.size(); i++) {
                            if (ooi.get(i).getId() == idAlreadyTracked) {
                                index = i;
                                break;
                            }
                        }
                        if(index!=-1) {
                            ooi.remove(index);
                        }
                        
                    }
                }

                if (ooi.size()==0) {
                    // No candidate to track but try search
                    indexSelectedObjective = 2;
                } else {
                    objective = new TrackingObjective(ooi, stationFrame, topoInertial, sensor);
                    break;
                }
            case 2:
                searchPossible = true;
                // make sure that tree does not get expanded by the same node that already exist among siblings
                for(Node sibling : leaf.getChildren()) {
                    ChanceNode chance = (ChanceNode)sibling;
                    
                    if (chance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                        searchPossible = false; // Search is already performed by another sibling
                    }
                }
                // Time until end of observation campaign
                /* leftT = this.endCampaign.durationFrom(leaf.getEpoch());
                if(leftT < scanStripe.getStripeT(numExpo)){
                    searchPossible = false; // Not enough time to complete search
                } */
                if(searchPossible) {
                    objective = new SearchObjective(stationFrame, scanStripe, numExpo, sensor);
                    break;
                } else {
                    return null; // Tasking not possible
                }

            default:
                throw new IllegalAccessError("Unknown objective.");
        }

        List<ObservedObject> restore = leaf.getPropEnvironment();
        AngularDirection pointing = 
            objective.setMicroAction(leaf.getEpoch(), leaf.getSensorPointing());
        leaf.setPropEnvironment(restore);
        if (Objects.isNull(pointing)) {
            // none of the considered targets was observable --> no expansion possible
            return null;
        }
        expandedChance = new ChanceNode(objective.getExecusionDuration(leaf.getEpoch()), 
                                        0., 0, objective, pointing, leaf);   
        expandedChance.setId(leaf.getId()+leaf.getChildren().size());
        //selected.add(toBeAdded);

        // Expand by Decision node too
        // Need to propagate the environment under the selected macro/micro action pair
        List<ObservedObject> propEnviroment = objective.propagateOutcome();  
        if (!Objects.isNull(propEnviroment)) {
            // tracking objective has been selected
            // Add object that was not considered for tracking back to propEnvironment
            for(int parent=0; parent<leaf.getPropEnvironment().size(); parent++) {
                long idParent = leaf.getPropEnvironment().get(parent).getId();
                boolean found = false;
                for(int child=0; child<propEnviroment.size(); child++) {
                    if (idParent == propEnviroment.get(child).getId()) {
                        found = true;
                    }
                }
                if(!found) {
                    ObservedObject notTargeted = 
                        new ObservedObject(idParent, leaf.getPropEnvironment().get(parent).getState(),
                                        leaf.getPropEnvironment().get(parent).getCovariance(), 
                                        leaf.getPropEnvironment().get(parent).getEpoch(), 
                                        leaf.getPropEnvironment().get(parent).getFrame());
                    propEnviroment.add(notTargeted);
                }
            }
        }

        // Update 
        //DecisionNode grandparent = (DecisionNode) leaf.getParent();
        double[] priorTimeResources = leaf.getTimeResources();
        double[] postTimeResources = new double[priorTimeResources.length];
        double[] postWeights = new double[leaf.getWeights().length];

        // Compute post observation duration
        double priorTobs = 0;
        for (int i=0; i<priorTimeResources.length; i++) {
            priorTobs += priorTimeResources[i];
        }
        AbsoluteDate[] obsTimeInterval = expandedChance.getExecutionDuration();
        double executionDuration = obsTimeInterval[1].durationFrom(obsTimeInterval[0]);
        double postTobs = priorTobs - executionDuration;

            // Update weights
            for (int i=0; i<postWeights.length; i++) {
            postWeights[i] = priorTimeResources[i]/postTobs;
        }

        AngularDirection sensorPointing;
        String objectiveType = objective.getClass().getSimpleName();
        if (objectiveType.equals("SearchObjective")) {
            // Update time resources
            postTimeResources[0] = priorTimeResources[0] - executionDuration;
            postTimeResources[1] = priorTimeResources[1];
            
            // Correct weight update for given objective
            postWeights[0] = postTimeResources[0]/postTobs;

            // Assign sensor pointing location
            List<AngularDirection> tasks = ((SearchObjective)expandedChance.getMacro()).getScheduleTopocentric();
            sensorPointing = tasks.get(tasks.size()-1);

        } else if(objectiveType.equals("TrackingObjective")) {
            // Update time resources
            postTimeResources[1] = priorTimeResources[1] - executionDuration;
            postTimeResources[0] = priorTimeResources[0];

            // Correct weight update for given objective
            postWeights[1] = postTimeResources[1]/postTobs;

            // Assign sensor pointing location
            sensorPointing = expandedChance.getMicro();
        }else {
            throw new IllegalAccessError("Unknown objective.");
        }
        // TODO: Try MCTS handling weight 
        postWeights = new double[]{0.5, 0.5};

        AbsoluteDate propEpoch = leaf.getEpoch().shiftedBy(executionDuration);
        expandedDecision = new DecisionNode(0., 0, sensorPointing, postWeights, 
                                                    postTimeResources, propEpoch, propEnviroment);  
        expandedChance.setChild(expandedDecision); 
        expandedDecision.setId(expandedChance.getId()+expandedChance.getChildren().size());
         
        
        //selected.add(toBeAdded);
        
        return expandedDecision;
    }

    /**
     * Roll out the decision tree until termination condition is met given by the end of the 
     * observation campaign. In case a searching action has been selected previously, an empty
     * list will be returned because there is no need for a simulation phase. 
     * 
     * @param leaf              Current leaf node of the decision tree.
     * @param campaignEndDate   End of observation campaign.
     * @return                  List of nodes that have been simulated during roll-out.
     */
    public List<Node> simulate(DecisionNode leaf, AbsoluteDate campaignEndDate) {

        List<ObservedObject> restore = leaf.getPropEnvironment();
        // Declare output
        List<Node> episode = new ArrayList<Node>(); // TODO: not necessary to store in an array because node holds all the descendants
        //episode.add(leaf);

        DecisionNode current = leaf;
        AbsoluteDate currentEndMeasEpoch = current.getEpoch();

        while(currentEndMeasEpoch.compareTo(campaignEndDate) <= 0) {
            episode.add(current.getParent());
            episode.add(current);
            current = expand(current, true); 
            if (Objects.isNull(current)) {
                return episode;
            }         
            currentEndMeasEpoch = current.getEpoch();
        }

/*         // If simulated leaf node is a searching objectiv, then it can be still added to the output
        if (((ChanceNode)current.getParent()).getMacro().getClass().getSimpleName().equals("SearchObjective")) {
            episode.add(current.getParent());
            episode.add(current);
        } */
       
        if(episode.size()>0) {
            episode.remove(0);
        }
        leaf.clearChildren();  
        leaf.setPropEnvironment(restore);     
        return episode;
    }

    /**
     * Update the state of every parent node along the episode from the initial node down to the 
     * newest expanded node, i.e. simulated nodes (including termination node) do not get added to  
     * the decision tree and do not need to get updated.
     * 
     * @param leaf              Current leaf node of the decision tree (not including simulated nodes).
     * @param last              Termination node.
     */
    public Node backpropagate(Node leaf, Node last) {

        DecisionNode lastDecision;
        ChanceNode lastChance;
        Node parent;
        if (Objects.isNull(last)) {
            //No simulation was performed
            DecisionNode fakeRoot = 
                new DecisionNode(0., 0, null, null, 
                                 ((DecisionNode)leaf.getParent().getParent()).getTimeResources(), 
                                 leaf.getParent().getParent().getEpoch(), null);
            lastChance = 
                new ChanceNode(((ChanceNode)leaf.getParent()).getExecutionDuration(), leaf.getParent().getUtility(), 
                                leaf.getParent().getNumVisits(), ((ChanceNode)leaf.getParent()).getMacro(), 
                                ((ChanceNode)leaf.getParent()).getMicro(), fakeRoot);
            lastDecision = 
                new DecisionNode(leaf.getUtility(), leaf.getNumVisits(), ((DecisionNode)leaf).getSensorPointing(), 
                                ((DecisionNode)leaf).getWeights(), ((DecisionNode)leaf).getTimeResources(), 
                                leaf.getEpoch(), ((DecisionNode)leaf).getPropEnvironment());
            Node.setParent(lastDecision, lastChance);
            parent = leaf;
            //leaf = leaf.getParent();
        } else {
            lastDecision = (DecisionNode) last;
            parent = lastDecision;
        }
        // Compute utility value of last node
        
        double[] spentResources = new double[lastDecision.getTimeResources().length];
        double[] initWeights = ((DecisionNode)this.initial).getWeights();
        double obsCampaignDuration = endCampaign.durationFrom(startCampaign);
        double timeSpentObserving = obsCampaignDuration;
        for(int i=0; i<lastDecision.getTimeResources().length; i++) {
            timeSpentObserving -= lastDecision.getTimeResources()[i];
        }
        double[] lastUtility = new double[2];
        double totalUtility = 0.;

        if(timeSpentObserving>obsCampaignDuration) {
            timeSpentObserving = obsCampaignDuration;
        }
       
        // How much time has been spent on each objective up until end of observation campaign
        while(parent.getEpoch().durationFrom(endCampaign) > 0.) {
            parent = parent.getParent();
        }
        DecisionNode grandParent;
        double untilEnd = 0.;
        if(parent.getParent().getClass().getSimpleName().equals("ChanceNode")) {
            System.out.println("Error occurs here");
            grandParent = (DecisionNode) parent;
        } else {
            grandParent = (DecisionNode) parent.getParent();
            
            if(((ChanceNode) parent).getMacro().getClass().getSimpleName().equals("SearchObjective")){
                untilEnd = endCampaign.durationFrom(parent.getEpoch());
            }
        }
        
        for(int i=0; i<lastUtility.length; i++) {
                   
            spentResources[i] = ((DecisionNode)this.initial).getTimeResources()[i] - grandParent.getTimeResources()[i];
            if(i==0) {
                spentResources[i] += untilEnd;
            }

            lastUtility[i] = FastMath.abs((spentResources[i]/timeSpentObserving) - initWeights[i]);
            totalUtility += lastUtility[i];
        }
        totalUtility = 1 - totalUtility;
        lastDecision.setUtility(totalUtility);

        if (!leaf.equals(this.initial)) {
            leaf.incrementNumVisits();
            double updatedUtility = leaf.getUtility() + lastDecision.getUtility();
            leaf.setUtility(updatedUtility); 
            return backpropagate(leaf.getParent(), lastDecision);
        } else {
            // Update root too
            this.initial.incrementNumVisits();
            double updatedUtility = this.initial.getUtility() + lastDecision.getUtility();
            this.initial.setUtility(updatedUtility);
            return this.initial;
        }
    }

    /**
     * Given the {@code current} node, the next child is selected based on the Upper 
     * Confidence Bound criteria.
     * 
     * @param current           Current node with children out of which the next one shall be 
     *                          selected.
     * @return                  Child node that maximises UCB criteria.
     */
    protected static Node selectChildUCB(Node current) {
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

    protected static Node selectChildRobustMax(Node current) {
        double robustMax = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;

        for (Node child : current.getChildren()){
            // Compute UCB 
            double v = child.getUtility();
            double n = child.getNumVisits();
            double sum = v + n;

            // search for child that maximises UCB
            if (sum>robustMax) {
                potentiallySelected = child;
                robustMax = sum;
            }
        }
        return potentiallySelected;
    }

        
    public static void main(String[] args) {
/*         List<Node> test = new ArrayList<Node>();
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
        } */

/*         Node test = new DecisionNode(C, 0, null, new double[]{0.3, 0.7}, null);
        System.out.println(test.getClass().getSimpleName().equals("DecisionNode"));
        DecisionNode convert = (DecisionNode) test;
        double[] weights = convert.getWeights();
        System.out.println(weights[0] + weights[1]); */
    }
}
