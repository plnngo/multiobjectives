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
    final static double alpha = 0.7;

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

    /** Maximal number of MCTS iterations. */
    final int iteration = 10;

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

  
    private Stripe computeScanStripe() {

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
            System.out.println("Num of visits " + initial.getNumVisits());

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
        outputUCB.add(initial);
        // Travers decision until leaf node
        while(!Objects.isNull(current) && current.getChildren().size() !=0){
            current = selectChildUCB(current);
            outputUCB.add(current);
        }

        current = initial;
        outputRobustMax.add(initial);
        // Travers decision until leaf node
        while(!Objects.isNull(current) && current.getChildren().size() !=0){
            current = selectChildRobustMax(current);
            outputRobustMax.add(current);
        }
        return outputUCB;
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


        // TODO: outsource l.229 - 259 to new function progressiveWidening()
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
                    leaf.getParent().incrementNumVisits();
                    leaf.incrementNumVisits();
                    return current;
                }
                expandable = true;
                List<Node> simulated = simulate(leaf, endCampaign);
                if (simulated.size() != 0) {
                    backpropagate(leaf, simulated.get(simulated.size()-1));
                } else {
                    backpropagate(leaf, null);
                }
                //children = current.getChildren();
            } 
            //return current;
        }

        current.incrementNumVisits();
        Node nextChild = current;
        while(nextChild.getChildren().size() != 0) {
            nextChild = selectChildUCB(nextChild);
            if (Objects.isNull(nextChild)) {
                // no time for further tasks
                return current;
            }
            nextChild.incrementNumVisits();
        }
        // Reached leaf node but not end of campaign yet --> progressiveWidening()
        children = nextChild.getChildren();
        // check progressive widening condition
        if(nextChild.getClass().getSimpleName().equals("DecisionNode")) {
            boolean expandable = true;
            while(children.size() <= FastMath.pow(nextChild.getNumVisits(), alpha) && expandable) {

                // allow expansion of new node
                DecisionNode leaf = expand((DecisionNode) nextChild, false);
                if (Objects.isNull(leaf)){
                    // objects not observable 
                    children = nextChild.getChildren();
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
                //children = current.getChildren();
            } 
            return current;
        }
        return current;
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
        double[] weights = leaf.getWeights();

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
                // make sure that tree does not get expanded by the same node that already exist among siblings
                for(Node sibling : leaf.getChildren()) {
                    ChanceNode chance = (ChanceNode)sibling;
                    
                    if (chance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                        searchPossible = false; // Search is already performed by another sibling
                    }
                }
                // Time until end of observation campaign
                double leftT = this.endCampaign.durationFrom(leaf.getEpoch());
                if(leftT < scanStripe.getStripeT(numExpo)){
                    searchPossible = false; // Not enough time to complete search
                }
                if(searchPossible) {
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

                objective = new TrackingObjective(ooi, stationFrame, topoInertial, sensor);
                /* // Need to propagate the environment under the selected macro/micro action pair
                propEnviroment = objective.propagateOutcome();  

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
                } */
                break;

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

        String objectiveType = objective.getClass().getSimpleName();
        if (objectiveType.equals("SearchObjective")) {
            // Update time resources
            postTimeResources[0] = priorTimeResources[0] - executionDuration;
            if(postTimeResources[0]<0.) {
                postTimeResources[0] = 0.1;
            }
            postTimeResources[1] = priorTimeResources[1];
            
            // Correct weight update for given objective
            postWeights[0] = postTimeResources[0]/postTobs;

        } else if(objectiveType.equals("TrackingObjective")) {
            // Update time resources
            postTimeResources[1] = priorTimeResources[1] - executionDuration;
            if(postTimeResources[1]<0.) {
                postTimeResources[1] = 0.1;
            }
            postTimeResources[0] = priorTimeResources[0];

            // Correct weight update for given objective
            postWeights[1] = postTimeResources[1]/postTobs;
        }else {
            throw new IllegalAccessError("Unknown objective.");
        }
        AbsoluteDate propEpoch = leaf.getEpoch().shiftedBy(executionDuration);
        expandedDecision = new DecisionNode(0., 0, expandedChance.getMicro(), postWeights, 
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

/*         if(((ChanceNode)leaf.getParent()).getMacro().getClass().getSimpleName()
                                         .equals("SearchObjective")) {
            return new ArrayList<Node>();
        } */
        //System.out.println("Simulation phase:");
        List<ObservedObject> restore = leaf.getPropEnvironment();
        // Declare output
        List<Node> episode = new ArrayList<Node>(); // TODO: not necessary to store in an array because node holds all the descendants
        episode.add(leaf);

        DecisionNode current = leaf;
        double measDuration = TrackingObjective.allocation 
                                + this.sensor.getSettlingT() 
                                + TrackingObjective.preparation 
                                + this.sensor.getExposureT()
                                + this.sensor.getReadoutT();
        AbsoluteDate currentEndMeasEpoch = current.getEpoch().shiftedBy(measDuration);

        while(currentEndMeasEpoch.compareTo(campaignEndDate) <= 0) {
            current = expand(current, true); 
            if (Objects.isNull(current)) {
                return episode;
            }         
            //System.out.println(((ChanceNode)current.getParent()).getMacro() + " at " + current.getEpoch()); 
            episode.add(current.getParent());
            episode.add(current);
            currentEndMeasEpoch = current.getEpoch().shiftedBy(measDuration);
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

        if (Objects.isNull(last)) {
            // no simulation was performed

            if (!leaf.equals(this.initial)) {

                // In case leaf is initial node
                leaf.incrementNumVisits();
                double[] leftResources = ((DecisionNode)leaf).getTimeResources();
                double[] initWeights = ((DecisionNode)this.initial).getWeights();
                double obsCampaignDuration = endCampaign.durationFrom(startCampaign);
                double[] lastUtility = new double[2];
                double totalUtility = 0.;
                for(int i=0; i<lastUtility.length; i++) {
                    if(initWeights[i] == 0) {
                        // objective not selected
                        lastUtility[i] = 0.;
                    } else {
                        // TODO: handle case when left resources is exactly zero
                        lastUtility[i] = initWeights[i] * obsCampaignDuration / leftResources[i];
                        totalUtility += lastUtility[i];
                    }
                }
                totalUtility = totalUtility / 1000;
                leaf.setUtility(totalUtility);
            }

            DecisionNode leafDecision = (DecisionNode) leaf;
            last = new DecisionNode(leaf.getUtility(), leaf.getNumVisits(), 
                                    ((ChanceNode)leaf.getParent()).getMicro(), 
                                    leafDecision.getWeights(), leafDecision.getTimeResources(), 
                                    leaf.getEpoch(), new ArrayList<ObservedObject>()); // TODO: fill propEnvironment with IOD results of detections
            leaf = leaf.getParent();
        }
        // Compute utility value of last node
        DecisionNode lastDecision = (DecisionNode) last;
        double[] leftResources = lastDecision.getTimeResources();
        double[] initWeights = ((DecisionNode)this.initial).getWeights();
        double obsCampaignDuration = endCampaign.durationFrom(startCampaign);
        double[] lastUtility = new double[2];
        double totalUtility = 0.;
        for(int i=0; i<lastUtility.length; i++) {
            if(initWeights[i] == 0) {
                // objective not selected
                lastUtility[i] = 0.;
            } else {
                // TODO: handle case when left resources is exactly zero
                lastUtility[i] = initWeights[i] * obsCampaignDuration / leftResources[i];
                totalUtility += lastUtility[i];
            }
        }
        totalUtility = totalUtility / 1000;
        last.setUtility(totalUtility);
        
        if (!leaf.equals(this.initial)) {
            leaf.incrementNumVisits();
            double updatedUtility = leaf.getUtility() + last.getUtility();
            leaf.setUtility(updatedUtility); 
            return backpropagate(leaf.getParent(), last);
        } else {
            // Update root too
            this.initial.incrementNumVisits();
            double updatedUtility = this.initial.getUtility() + last.getUtility();
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
