package sensortasking.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Map;

import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import lombok.Getter;
import sensortasking.stripescanning.Stripe;
import sensortasking.stripescanning.Tasking;
import tools.OptimisingVector;
import tools.WeightedRandomNumberPicker;

@Getter
public class MultiObjectiveMcts {

    /** Root node. */
    DecisionNode initial;

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
                              AbsoluteDate start, AbsoluteDate end, String stationName,
                              List<ObservedObject> trackedObjects, List<ObservedObject> detectedObjects,
                              Sensor sensor) {

        this.initial = (DecisionNode) descisionTree;
        MultiObjectiveMcts.objectives = objectives;
        this.startCampaign = start;
        this.endCampaign = end;

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);

        this.stationFrame = new TopocentricFrame(earth, sensor.getPosition(), stationName);
        this.sensor = sensor;
        this.scanStripe = computeScanStripe();
    }

  
    public Stripe computeScanStripe() {

        Tasking survey = new Tasking(sensor, this.startCampaign, this.endCampaign, this.numExpo);
        Stripe[] stripes = survey.computeScanStripes();

        // TODO: select fixed stripe that does not enter earth shadow
        return stripes[1];
    }

    public List<Node> run(int iterations) {

        //List<Node> outputUCB = new ArrayList<Node>();
        //List<Node> outputRobustMax = new ArrayList<Node>();

        for(int i=0; i<iterations; i++) {  
            List<Node> outputRobustMaxRatio = new ArrayList<Node>();

            if (i==3000) {
                continue;
            } 
            System.out.println("Iteration: " + i);
            selectNew(this.initial);
            // Retrieve pointing strategy UCB
            Node current = initial;

            //outputRobustMax.add(initial);
            outputRobustMaxRatio.add(initial);
            // Travers decision until leaf node
            while(!Objects.isNull(current) && current.getChildren().size() !=0){
                current = selectChildRobustMax(current);
                //current = selectChildRobustMaxRatio(current);
                //outputRobustMax.add(current);
                outputRobustMaxRatio.add(current);
            }
            if(i==iterations-1) {
                return outputRobustMaxRatio;
            } else {
                for(Node currentNode : outputRobustMaxRatio) {
                    if (currentNode.getClass().getSimpleName().equals("ChanceNode")) {
                        String objective = ((ChanceNode) currentNode).getMacro().getClass().getSimpleName();
                        if (objective.equals("TrackingObjective")) {
                            long id = ((TrackingObjective)((ChanceNode) currentNode).getMacro())
                                                                                        .getLastUpdated();

                            System.out.print(id + " ");
                            
                        } else {
                            System.out.print( " S ");
                        }
                    } else {
                        continue;
                    }
                }
                outputRobustMaxRatio.clear();
            }
        }

        return null;
        //return outputRobustMax;
    }

    public Node selectChildRobustMaxRatio(Node current) {
        double robustMax = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;

        for (Node child : current.getChildren()){
            // Compute UCB 
            double v = child.getUtility();
            double n = child.getNumVisits();
            double ratio = v / n;

            // search for child that maximises the sum of visits and values
            if (ratio>robustMax) {
                potentiallySelected = child;
                robustMax = ratio;
            }
        }
        return potentiallySelected;
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
                //System.out.println("After expansion leaf epoch: " + leaf.getEpoch().toString());
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

        if (current.getId()==0 && current.getChildren().size()==0) {
            progressiveWidening(current);
            return current;
        }

        Node nextChild = current;
        //while(nextChild.getChildren().size() != 0) {
        while(this.endCampaign.durationFrom(current.getEpoch()) > 0.){
            boolean widening = progressiveWidening(nextChild);
            if (nextChild.getEpoch().compareTo(endCampaign) >= 0) {
                // reached end of campaign
                return current;
            } else if (!widening){
                do{
                    nextChild = selectChildUCB(nextChild);
                } while (!nextChild.getClass().getSimpleName().equals("DecisionNode"));
                // progressive widening not possible but not end of campaign yet
                continue;
            } else if (widening) {
                // break out of while loop and start new MCTS iteration from root node
                break;
            }
        }
        // Reached leaf node but not end of campaign yet --> progressiveWidening() --> TODO: remove last line
        //widening = progressiveWidening(nextChild);
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
                    //expandable = false; // TODO: still backpropagate!
                    do{
                        current = selectChildUCB(current);
                        children = current.getChildren();
                    } while (!current.getClass().getSimpleName().equals("DecisionNode"));

                    continue;

                } else if (leaf.getEpoch().compareTo(endCampaign) >= 0) {
                    // already reached end of campaign
                    if (((ChanceNode)leaf.getParent()).getMacro().getClass().getSimpleName().equals("TrackingObjective")) {     
                        Node grand = leaf.getParent().getParent();   
                        grand.removeChild(leaf.getParent());
                        Node nextChild = grand;
                        while(nextChild.getChildren().size() != 0) {
                            nextChild = selectChildUCB(nextChild);
                        }
                        leaf = (DecisionNode) nextChild;      
                    } else if (((ChanceNode)leaf.getParent()).getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                        Node grand = leaf.getParent().getParent();
                        if (grand.getEpoch().compareTo(endCampaign) >= 0) {
                            grand.removeChild(leaf.getParent());
                            return false;
                        }

                    }
                }
                expandable = true;
                List<Node> simulated = simulate(leaf, endCampaign);
                if (simulated.size() > 1) {
                    backpropagate(leaf, simulated.get(simulated.size()-1));
                } else {
                    backpropagate(leaf, null);
                }
                return true;
            } 
        }
        return false;
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

        ChanceNode expandedChance = null;
        DecisionNode expandedDecision = null;
        List<ObservedObject> restore = new ArrayList<>();
        for(ObservedObject target : leaf.getEnvironment().getStateTracking()) {
            ObservedObject copy = new ObservedObject(target.getId(), target.getState(), 
                                                     target.getCovariance(), target.getEpoch(), 
                                                     target.getFrame());
            restore.add(copy);
        }

        // Expand by Chance node first
        // Need to sample a new pair of macro and micro action
        double[] weights = new double[]{1., 1.};

        // Generate array filled with indexes representing the objective IDs
        int[] indexObjective = new int[weights.length];
        for (int i=0; i<weights.length; i++) {
            indexObjective[i] = i;
        }
        int indexSelectedObjective = 
            WeightedRandomNumberPicker.pickNumber(indexObjective, weights);
        Objective objective;
        AngularDirection pointing = null;
        //List<ObservedObject> propEnviroment = new ArrayList<ObservedObject>();
        switch (indexSelectedObjective) {
            case 0:
                boolean searchPossible = true;
                boolean searchAlreadyPerformed = false;
                // make sure that tree does not get expanded by the same node that already exist among siblings
                for(Node sibling : leaf.getChildren()) {
                    ChanceNode chance = (ChanceNode)sibling;
                    
                    if (chance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                        searchAlreadyPerformed = true; // Search is already performed by another sibling TODO: break from for loop
                    }
                }
                // Time until end of observation campaign
                double leftT = this.endCampaign.durationFrom(leaf.getEpoch());
                List<Integer> completedSearchTasks = leaf.getEnvironment().getStateSearching();

                if(leftT < scanStripe.getStripeT(numExpo)){
                    searchPossible = false; // Not enough time to complete search TODO: indexSelectedObjective = 1;
                } 
                                       
                if(searchPossible && !searchAlreadyPerformed) {

                    // Increment stripe scan tasks by one
                    completedSearchTasks.set(0, completedSearchTasks.get(0) + 1);
                    objective = new SearchObjective(completedSearchTasks, stationFrame, 
                                                    scanStripe, numExpo, sensor);
                    break;
                } else if(!searchAlreadyPerformed && leaf.getTimeResources()[1] < TrackingObjective.allocation 
                                                                                    + this.sensor.getSettlingT() 
                                                                                    + TrackingObjective.preparation 
                                                                                    + this.sensor.getExposureT()) {
                    objective = new SearchObjective(completedSearchTasks, stationFrame, scanStripe, numExpo, sensor);
                    break;
                } else {
                    indexSelectedObjective = 1;
                }

            case 1:
                // Macro action = track
                List<ObservedObject> ooi = new ArrayList<>(restore);

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

                objective = new TrackingObjective(ooi, sensor, this.endCampaign);
                leaf.setEpochSensorPointing(leaf.getEpoch());
                pointing = objective.setMicroAction(leaf.getEpoch(), leaf.getSensorPointing());

                if (Objects.isNull(pointing)) {
                    // No candidate to track but try search
                    indexSelectedObjective = 2;
                    pointing = null;
                } else {
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
                if(searchPossible) {
                    completedSearchTasks = leaf.getEnvironment().getStateSearching();

                    // Increment stripe scan tasks by one
                    completedSearchTasks.set(0, completedSearchTasks.get(0) + 1);   
                    objective = new SearchObjective(completedSearchTasks, stationFrame, scanStripe, numExpo, sensor);
                    break;
                } else {
                    return null; // Tasking not possible
                }

            default:

                throw new IllegalAccessError("Unknown objective.");
        }


        if(Objects.isNull(pointing)) {
            pointing = objective.setMicroAction(leaf.getEpoch(), leaf.getSensorPointing());
        }

        if (Objects.isNull(pointing)) {
            // none of the considered targets was observable --> no expansion possible
            return null;
        }
        expandedChance = new ChanceNode(objective.getExecusionDuration(leaf.getEpoch()), 
                                        0., 0, objective, pointing, leaf, 
                                        this.initial.incrementIdCounter());      
        // Update 
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

        if(objective instanceof TrackingObjective) {
            List<ObservedObject> propEnviroment = new ArrayList<ObservedObject>();
            for(ObservedObject obj: (List<ObservedObject>)objective.propagateOutcome()) {
                ObservedObject copy = new ObservedObject(obj.getId(), obj.getState(), 
                                                         obj.getCovariance(), obj.getEpoch(), 
                                                         obj.getFrame());
                propEnviroment.add(copy);
            }
            for(int parent=0; parent<leaf.getEnvironment().getStateTracking().size(); parent++) {
                long idParent = leaf.getEnvironment().getStateTracking().get(parent).getId();
                boolean found = false;
                for(int child=0; child<propEnviroment.size(); child++) {
                    if (idParent == ((ObservedObject)propEnviroment.get(child))
                                                                   .getId()) {
                        found = true;
                    }
                }
                if(!found) {
                    ObservedObject notTargeted = 
                        new ObservedObject(idParent, leaf.getEnvironment().getStateTracking().get(parent).getState(),
                                        leaf.getEnvironment().getStateTracking().get(parent).getCovariance(), 
                                        leaf.getEnvironment().getStateTracking().get(parent).getEpoch(), 
                                        leaf.getEnvironment().getStateTracking().get(parent).getFrame());
                    propEnviroment.add(notTargeted);
                }
            }
            PropoagatedEnvironment environment = 
                new PropoagatedEnvironment(propEnviroment, 
                                           leaf.getEnvironment().stateSearching);
            expandedDecision = new DecisionNode(0., 0, sensorPointing, postWeights, 
                                                postTimeResources, propEpoch, environment,
                                                this.initial.incrementIdCounter());  
        } else if (objective instanceof SearchObjective) {
            // searching objective has been selected TODO: hard copy of propagatedOutcome might be necessary
            // for now, only stripe scan is performed TODO: implement bullseye
            List<Integer> propEnviroment = objective.propagateOutcome();
            //propEnviroment.set(0, (Integer)propEnviroment.get(0) + 1);
            PropoagatedEnvironment environment = 
                new PropoagatedEnvironment(leaf.getEnvironment().getStateTracking(), 
                                           propEnviroment);
            expandedDecision = new DecisionNode(0., 0, sensorPointing, postWeights, 
                                                postTimeResources, propEpoch, environment,
                                                this.initial.incrementIdCounter());

        } else {
            // other objective was selected
        }

        expandedChance.setChild(expandedDecision); 
/*         expandedChance.setUtilityVec(new double[2]);
        expandedDecision.setUtilityVec(new double[2]); */
        
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

        //List<ObservedObject> restore = leaf.getEnvironment().getStateTracking();
        List<ObservedObject> restore = new ArrayList<>();
        for(ObservedObject target : leaf.getEnvironment().getStateTracking()) {
            ObservedObject copy = new ObservedObject(target.getId(), target.getState(), 
                                                     target.getCovariance(), target.getEpoch(), 
                                                     target.getFrame());
            restore.add(copy);
        }
        // Declare output
        List<Node> episode = new ArrayList<Node>(); // TODO: not necessary to store in an array because node holds all the descendants
        //episode.add(leaf);

        DecisionNode current = leaf;
        AbsoluteDate currentEndMeasEpoch = current.getEpoch();

        while(currentEndMeasEpoch.compareTo(campaignEndDate) <= 0) {

            episode.add(current);
            current = expand(current, true); 

            if (Objects.isNull(current)) {
                return episode;
            }         
            currentEndMeasEpoch = current.getEpoch();
        }
      
        leaf.clearChildren();  
        leaf.getEnvironment().setStateTracking(restore);   // TODO: not necessary  

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
    public Node backpropagateTimeUtility(Node leaf, Node last) {

        DecisionNode lastDecision;
        ChanceNode lastChance;
        Node parent;
        if (Objects.isNull(last)) {
            //No simulation was performed
            DecisionNode fakeRoot = 
                new DecisionNode(0., 0, null, null, 
                                 ((DecisionNode)leaf.getParent().getParent()).getTimeResources(), 
                                 leaf.getParent().getParent().getEpoch(), null,
                                 this.initial.incrementIdCounter());
            lastChance = 
                new ChanceNode(((ChanceNode)leaf.getParent()).getExecutionDuration(), leaf.getParent().getUtility(), 
                                leaf.getParent().getNumVisits(), ((ChanceNode)leaf.getParent()).getMacro(), 
                                ((ChanceNode)leaf.getParent()).getMicro(), fakeRoot,
                                this.initial.incrementIdCounter());
            lastDecision = 
                new DecisionNode(leaf.getUtility(), leaf.getNumVisits(), ((DecisionNode)leaf).getSensorPointing(), 
                                ((DecisionNode)leaf).getWeights(), ((DecisionNode)leaf).getTimeResources(), 
                                leaf.getEpoch(), ((DecisionNode)leaf).getEnvironment(),
                                this.initial.incrementIdCounter());
            Node.setParent(lastDecision, lastChance);
            parent = leaf;
        } else {
            lastDecision = (DecisionNode) last;
            parent = lastDecision;
        }
        // Compute utility value of last node
        double[] utilityVec = computeUtilityVector(lastDecision, (DecisionNode)leaf);

        // Compare with other solutions
        Map<Long, double[]> otherUtilities = this.initial.getAllUtilityVecs();

        DecisionNode grand = (DecisionNode)leaf.getParent().getParent();
        
        // Number of solutions current leaf dominates
        int nDom = 0;
        
        if(otherUtilities.containsKey(grand.getId())) {
            // new utility vector should replace old leaf
            this.initial.removeUtilityVec(grand.getId());
        } 
        List<double[]> otherLeafs = new ArrayList<double[]>(this.initial.getAllUtilityVecs().values());

        if(otherLeafs.size()>0) {
            int dim = otherLeafs.get(0).length;
            OptimisingVector opt = new OptimisingVector(otherLeafs, 0);

            // search utility vectors dominate by maximising
            boolean[] domMax = new boolean[dim];
            for(int i=0; i<dim; i++) {
                domMax[i] = true;
            }
            List<double[]> dominating = opt.getDominatingVecs(utilityVec, domMax, 0);
            if(dominating.size() != 0) {
                nDom = dominating.size() * (-1);
            }
        }
        // add new utility vector to list of utilities
        this.initial.addUtilityVec(leaf.getId(), utilityVec);
        
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
        
        if (Objects.isNull(last)) {
            //No simulation was performed
            List<ObservedObject> fakeObjs = new ArrayList<ObservedObject>();
            for(ObservedObject obj : this.initial.getEnvironment().getStateTracking()) {
                ObservedObject copy = new ObservedObject(obj.getId(), obj.getState(), 
                                                         obj.getCovariance(), obj.getEpoch(), 
                                                         obj.getFrame());
                fakeObjs.add(copy);
            }
            List<Integer> fakeSearchTask = new ArrayList<Integer>();
            for(Integer task : this.initial.getEnvironment().getStateSearching()) {
                Integer copy = new Integer(task);
                fakeSearchTask.add(copy);
            }

            DecisionNode fakeRoot = 
                new DecisionNode(0., 0, null, null, 
                                 ((DecisionNode)leaf.getParent().getParent()).getTimeResources(), 
                                 leaf.getParent().getParent().getEpoch(), 
                                 new PropoagatedEnvironment(fakeObjs, fakeSearchTask),
                                 this.initial.incrementIdCounter());
            lastChance = 
                new ChanceNode(((ChanceNode)leaf.getParent()).getExecutionDuration(), leaf.getParent().getUtility(), 
                                leaf.getParent().getNumVisits(), ((ChanceNode)leaf.getParent()).getMacro(), 
                                ((ChanceNode)leaf.getParent()).getMicro(), fakeRoot,
                                this.initial.incrementIdCounter());
            lastDecision = 
                new DecisionNode(leaf.getUtility(), leaf.getNumVisits(), ((DecisionNode)leaf).getSensorPointing(), 
                                ((DecisionNode)leaf).getWeights(), ((DecisionNode)leaf).getTimeResources(), 
                                leaf.getEpoch(), ((DecisionNode)leaf).getEnvironment(),
                                this.initial.incrementIdCounter());
            Node.setParent(lastDecision, lastChance);
        } else {
            lastDecision = (DecisionNode) last;
        }
        // Compute utility value of leaf node
        double[] utilityVec = computeUtilityVector(lastDecision, (DecisionNode)leaf);
        
        // Number of solutions current leaf dominates
        int nDom = 0;
        
        // check if leaf has any siblings
        DecisionNode grand = (DecisionNode)leaf.getParent().getParent();
        if(grand.getChildren().size()>1) {
            // leaf has siblings --> need to add to list of vecs

        } else {
            // leaf does not have siblings --> need to replace parental vec in list of vecs
            this.initial.removeUtilityVec(grand.getId());
        }
        List<double[]> otherLeafs = new ArrayList<double[]>(this.initial.getAllUtilityVecs().values());

        if(otherLeafs.size()>0) {
            int dim = otherLeafs.get(0).length;
            OptimisingVector opt = new OptimisingVector(otherLeafs, 0);

            // search utility vectors dominate by maximising
            boolean[] domMax = new boolean[dim];
            for(int i=0; i<dim; i++) {
                domMax[i] = true;
            }
            List<double[]> dominating = opt.getDominatingVecs(utilityVec, domMax, 0);
            if(dominating.size() != 0) {
                nDom = dominating.size() * (-1);
            }
        }
        // add new utility vector to list of utilities
        this.initial.addUtilityVec(leaf.getId(), utilityVec);
        
        //lastDecision.setUtility(nDom);

        Node current = leaf;

        while (!current.equals(this.initial)) {
            current.incrementNumVisits();
            double updatedUtility = current.getUtility() + nDom;
            current.setUtility(updatedUtility);
            double[] preUtilityVec = current.getUtilityVec();
            double[] postUtilityVec = new double[preUtilityVec.length];
            for(int i=0; i<preUtilityVec.length; i++) {
                postUtilityVec[i] = preUtilityVec[i] + utilityVec[i];
            }
            current.setUtilityVec(postUtilityVec);
            current = current.getParent();
        }

        this.initial.incrementNumVisits();
        double updatedUtility = this.initial.getUtility() + nDom;
        this.initial.setUtility(updatedUtility);
        double[] preUtilityVec = this.initial.getUtilityVec();
        double[] postUtilityVec = new double[preUtilityVec.length];
        for(int i=0; i<preUtilityVec.length; i++) {
            postUtilityVec[i] = preUtilityVec[i] + utilityVec[i];
        }
        return this.initial;
    }

    private double[] computeUtilityVector(DecisionNode last, DecisionNode leaf) {

        // Compute tracking reward
        double[] trackReward = computeTrackReward(last);
        
        // Compute searching reward
        //double searchReward = computeSearchReward(last, leaf); TODO: function errornous because rSearch sometimes not zero
        double searchReward = 0.;

        double[] out = new double[trackReward.length + 1];
        out[0] = searchReward;
        for (int i=1; i<trackReward.length+1; i++) {
            out[i] = trackReward[i-1];
        }

        // Build up utility vector from macro action rewards                
        //return new double[]{searchReward, trackReward};
        return out;
    }

    protected double[] computeTrackReward(DecisionNode last) {
        
        // Compute common epoch
        List<ObservedObject> trackedObjs = last.getEnvironment().getStateTracking();

        // Initialise output
        double[] out = new double[trackedObjs.size()];

        // Propagate all targets from their intial state towards common epoch with Kepler dynamics
        List<ObservedObject> targetsInitial = 
            ((DecisionNode)this.initial).getEnvironment().getStateTracking();
        List<ObservedObject> targetsPredicted = 
            ObservedObject.propagateTargets(targetsInitial, this.endCampaign);

        // Propagate all targets from their updated final state towards common epoch
        //List<ObservedObject> targetsUpdated = last.getEnvironment().getStateTracking();
        List<ObservedObject> targetsFinal = 
            ObservedObject.propagateTargets(trackedObjs, this.endCampaign);

        // Calculate information gain
        if(targetsPredicted.size() != targetsFinal.size()) {
            throw new IllegalArgumentException("Information gain cannot be computed due to " 
                                                + "dimension error in targets.");
        }
        double accumulatedIG = 0;

        for(int i=0; i<targetsPredicted.size(); i++) {
            int j=0;
            while(j<targetsFinal.size()) {

                // Make sure that ID of objects are the same when computing information gain
                if(targetsPredicted.get(i).getId() != targetsFinal.get(j).getId()) {
                    // Move to next object in targetFinals                                                                          
                    j++;
                } else {
                    // Same ID found
                    out[i] = TrackingObjective.computeInformationGain(targetsPredicted.get(i), 
                                                                      targetsFinal.get(j));
                    accumulatedIG += out[i];
                        
                    // No need to continue searching in targetFinals
                    targetsFinal.remove(j);
                    j=0;
                    break;
                }
            }
        }

        return out;
    }


    /**
     * Return number of dominating solutions with respect to the new {@code leaf} node.
     * 
     * @param last              (Simulated) termination node.
     * @param leaf              Current leaf node.
     * @return                  Number of dominating solutions.
     */
    protected double computeSearchReward(DecisionNode last, DecisionNode leaf) {
        double[] weightsSearch = this.initial.getWeightsSearch();
        List<Integer> completedSearchTasks = last.getEnvironment().getStateSearching();
        int numTotalSearchTaskCompleted = 0;
        for(int i=0; i<completedSearchTasks.size(); i++) {
            numTotalSearchTaskCompleted += completedSearchTasks.get(i);
        }
        
        // Compute discrepance vector of leaf node
        double[] discrepance = new double[2];
        for(int i=0; i<completedSearchTasks.size(); i++){
            if (numTotalSearchTaskCompleted == 0) {
                discrepance[i] = weightsSearch[i];
            } else {
                discrepance[i] = 
                FastMath.abs(((double)completedSearchTasks.get(i)/numTotalSearchTaskCompleted) 
                                - weightsSearch[i]);
            }
            
        }

        DecisionNode grand = (DecisionNode)leaf.getParent().getParent();
        Map<Long, double[]> otherSearch = this.initial.getSearchDiscrepancyVecs();
        
        // check if leaf has any siblings
        if(grand.getChildren().size()>1) {
            // leaf has siblings --> need to add to list of vecs

        } else {
            // leaf does not have siblings --> need to replace parental vec in list of vecs
            otherSearch.remove(Long.valueOf(grand.getId()));

        }
        List<double[]> vecs = new ArrayList<double[]>(otherSearch.values());

        double searchReward = 0;
        if(vecs.size()>0) {
            int dim = vecs.get(0).length;
            OptimisingVector opt = new OptimisingVector(vecs, 0);

            // search utility vectors dominate by minimising
            boolean[] domMin = new boolean[dim];
            for(int i=0; i<dim; i++) {
                domMin[i] = false;
            }
            List<double[]> dominating = opt.getDominatingVecs(discrepance, domMin, 0);
            if(dominating.size() != 0) {
                System.out.println("error");
                searchReward = dominating.size() * (-1);
            }

        } 
        this.initial.addSearchDiscrepancyVec(leaf.getId(), discrepance);
        return searchReward;
    }


    /**
     * Given the {@code current} node, the next child is selected based on the Upper 
     * Confidence Bound criteria.
     * 
     * @param current           Current node with children out of which the next one shall be 
     *                          selected.
     * @return                  Child node that maximises UCB criteria or parent node, in case no 
     *                          children exist.
     */
    protected static Node selectChildUCB(Node current) {
        double weight = 1./3.;
        double maxUcb = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        double nP = current.getNumVisits();

        if(current.getChildren().size() == 0) {
            return null;
        }
        List<double[]> utilityChildrenNorm = 
            normaliseUtilityChildren(current.getChildren(), weight);
                
        for (int i=0; i<current.getChildren().size(); i++){
            double[] removedUtility = utilityChildrenNorm.remove(0);
            
            // Compute UCB 
            double n = current.getChildren().get(i).getNumVisits();
            //double[] utilityVec = current.getChildren().get(i).getUtilityVec();
/*             double totalRewardNorm = 0.;

            // Normalise utility vector by number of visits
            double[] utilityNorm = new double[utilityVec.length * 2 -1];
            for(int j=0; j<utilityNorm.length; j++) {
                if(j<utilityVec.length) {
                    utilityNorm[j] = utilityVec[j] / n;
                    totalRewardNorm += utilityNorm[j];
                } else {
                    double rewardNorm = utilityNorm[j-utilityNorm.length+1];
                    utilityNorm[j] = FastMath.abs((rewardNorm/totalRewardNorm) - weight);
                }
            } */

            OptimisingVector opt = new OptimisingVector(utilityChildrenNorm, removedUtility.length - 1);
            List<double[]> domVecs = 
                opt.getDominatingVecs(removedUtility, 
                                      new boolean[]{true, true, true, true, false, false, false}, 
                                      0);
            int utility = - domVecs.size();
            double ucb = utility + C * FastMath.sqrt(FastMath.log(nP)/n);
            utilityChildrenNorm.add(removedUtility);

            // search for child that maximises UCB
            if (ucb>maxUcb) {
                potentiallySelected = current.getChildren().get(i);
                maxUcb = ucb;
            }
        }
        return potentiallySelected;
    }

    private static List<double[]> normaliseUtilityChildren(List<Node> children, double weight) {
        List<double[]> utilityChildrenNorm = new ArrayList<double[]>();
        for (Node child : children) {
            double[] utilityNorm = new double[child.getUtilityVec().length * 2 -1];
            double totalRewardNorm = 0.;
            for(int i=0; i<utilityNorm.length; i++) {
                
                if (i<child.getUtilityVec().length) {
                    utilityNorm[i] = child.getUtilityVec()[i] / child.getNumVisits();
                    totalRewardNorm += utilityNorm[i];
                } else {
                    if(totalRewardNorm==0){
                        utilityNorm[i] = weight;
                    } else {
                        double rewardNorm = utilityNorm[i-child.getUtilityVec().length+1];
                        utilityNorm[i] = FastMath.abs((rewardNorm/totalRewardNorm) - weight);
                    }    
                }
            }
            utilityChildrenNorm.add(utilityNorm);
        }
        return utilityChildrenNorm;
    }


    protected static Node selectChildRobustMax(Node current) {
        double robustMax = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        
        // Reevaluate utility for every child
        double weight = 1./3.;
        List<double[]> utilityChildrenNorm = normaliseUtilityChildren(current.getChildren(), weight);
        
/*         for (Node child : current.getChildren()) {
            double[] utilityNorm = new double[child.getUtilityVec().length];
            for(int i=0; i<utilityNorm.length; i++) {
                utilityNorm[i] = child.getUtilityVec()[i] / child.getNumVisits();
            }
            utilityChildrenNorm.add(utilityNorm);
        } */

        for (Node child : current.getChildren()){
            double[] removedUtility = utilityChildrenNorm.remove(0);
            double n = child.getNumVisits();
            OptimisingVector opt = new OptimisingVector(utilityChildrenNorm, removedUtility.length - 1);
            List<double[]> domVecs = 
                opt.getDominatingVecs(removedUtility, 
                                      new boolean[]{true, true, true, true, false, false, false}, 
                                      0);
            int v = - domVecs.size();
            double sum = v + n;

            // search for child that maximises the sum of visits and values
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
