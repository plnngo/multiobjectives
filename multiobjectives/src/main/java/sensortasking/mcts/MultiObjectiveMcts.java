package sensortasking.mcts;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Random;
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

import benchtest.Car;
import benchtest.CarTrackingObjective;
import benchtest.Filter;
import benchtest.RewardFunction;
import lombok.Getter;
import sensortasking.stripescanning.Stripe;
import sensortasking.stripescanning.Tasking;
import tools.OptimisingVector;

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
    //static double C = 1.e50;
    static double C = 10000;

    /** Discount factor. */
    final double discount = 1.;

    /** Topocentric horizon frame. */
    final TopocentricFrame stationFrame;

    /** Earth centered Earth fixed frame. */
    final Frame j2000 = FramesFactory.getEME2000();

    /** Tuning parameter (0;1) for progressive widening */
    double[] alphaDepth = new double[]{17./134., 6096./44899, 24./151., 1.};

    /** Stripe for searching objective. */
    final Stripe scanStripe; 

    /** Number of exposures within stripe scanning algorithm. */
    final int numExpo = 5;

    /** Observation station (TODO: implement sensor for tracking objective). */
    final Sensor sensor;

    /** Maximum depth of decision in theory. */
    final double dmax;

    /** Minimal time duration requested by user that should be spent on time. */
    final double userSearchTrequested = 0.1;

    final boolean orbitMode = false;        // else car mode

    /** Cars propagated to end without measurement updates. */
    final List<ObservedObject> predictedCars = new ArrayList<ObservedObject>();

    final RewardFunction reward;


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
                              Sensor sensor, RewardFunction selectedReward) {

        this.initial = (DecisionNode) descisionTree;
        MultiObjectiveMcts.objectives = objectives;
        this.startCampaign = start;
        this.endCampaign = end;
        this.reward = selectedReward;

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);

        this.stationFrame = new TopocentricFrame(earth, sensor.getPosition(), stationName);
        this.sensor = sensor;
        this.scanStripe = computeScanStripe();

        double minTaskT = TrackingObjective.allocation + this.sensor.getSettlingT() 
                            + TrackingObjective.preparation + this.sensor.getExposureT() 
                            + this.sensor.getReadoutT();
        double campaignT = endCampaign.durationFrom(startCampaign);
        this.dmax = FastMath.ceil(campaignT/minTaskT) - 1;

        // If tracking cars
        if (!orbitMode) {
            for (ObservedObject obj : trackedObjects) {
                Car car = (Car) obj;
                Filter est = new Filter();
                /* double simMeas = 
                    CarTrackingObjective.generateRangeMeasurement(campaignT, car.getStateArray()); */
                /* double simMeas = 
                    CarTrackingObjective.generateBearingMeasurement(campaignT, car.getStateArray(), car); */
                est.run_ckf(car.getStateArray(), car.getCov(), car.getTime(), campaignT);
                Car pred = new Car(car.getIdentifier(), est.getStatePred(), 
                                   est.getCovPred(), campaignT);
                predictedCars.add(pred);
            }
        }
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

            System.out.println("Iteration: " + i);
            selectNew(this.initial);

            /* if (i==130) {
                DecisionNode current = (DecisionNode)this.initial;
                List<Map.Entry<String, Double>> branches = extractBranches(this.initial, "", 0.0);
                double maxReward = Double.NEGATIVE_INFINITY;
                for (Map.Entry<String, Double> entry : branches) {
                    System.out.println("Branch " + entry.getKey() + " has reward " + entry.getValue());
                    if (entry.getValue() > maxReward) {
                        maxReward = entry.getValue();
                    }
                }

                // Step 2: Collect all entries with that reward
                List<Map.Entry<String, Double>> bestBranches = new ArrayList<>();
                for (Map.Entry<String, Double> entry : branches) {
                    if (entry.getValue() == maxReward) {
                        bestBranches.add(entry);
                    }
                }

                // Step 3: Print them
                System.out.printf("Max reward: %.2f%n", maxReward);
                for (Map.Entry<String, Double> entry : bestBranches) {
                    System.out.printf("Best branch: %s with reward %.2f%n", entry.getKey(), entry.getValue());
                }
            } else if (i==499) {
                System.out.println("Break");
            } */
            // Retrieve pointing strategy UCB
            Node current = initial;

            //outputRobustMax.add(initial);
            outputRobustMaxRatio.add(initial);
            // Travers decision until leaf node
            while(!Objects.isNull(current) && current.getChildren().size() !=0){
                //current = selectChildRobustMax(current);    //TODO: utility is still null
                current = selectChildMaxUtility(current);
                outputRobustMaxRatio.add(current);
            }
            if(i==iterations-1) {
                return outputRobustMaxRatio;
            } else {
                for(Node currentNode : outputRobustMaxRatio) {
                    if (currentNode.getClass().getSimpleName().equals("ChanceNode")) {
                        String objective = ((ChanceNode) currentNode).getMacro().getClass().getSimpleName();
                        if (objective.equals("CarTrackingObjective")) {
                            char id = ((CarTrackingObjective)((ChanceNode) currentNode).getMacro())
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
    }

    private List<Map.Entry<String, Double>> extractBranches(Node current, String path, double accumulatedReward) {
        List<Map.Entry<String, Double>> result = new ArrayList<>();

        // Base case: leaf node
        if (current.getChildren().isEmpty()) {
            result.add(new AbstractMap.SimpleEntry<>(path, accumulatedReward));
            return result;
        }

        for (Node child : current.getChildren()) {
            if (child instanceof ChanceNode) {
                ChanceNode chanceChild = (ChanceNode) child;
                CarTrackingObjective macro = (CarTrackingObjective) chanceChild.getMacro();
                char decisionChar = macro.getLastUpdated();
                List<Node> children = child.getParent().getChildren();
                /* double regret = 0.;
                for (Node sibling : children) {
                    ChanceNode chanceSibling = ((ChanceNode) sibling);
                    char other = ((CarTrackingObjective)chanceSibling.getMacro()).getLastUpdated();
                    if (other != decisionChar) {
                        regret = ((CarTrackingObjective)chanceSibling.getMacro()).getRegret();
                    }
                } */
                double immediateReward = macro.getLastUpdatedIG();
                //double regret = chanceChild.getUtilityVec()[1];

                String newPath = path + decisionChar;
                double newAccumulatedReward = accumulatedReward + immediateReward;
                //double newAccumulatedReward = accumulatedReward + regret;


                result.addAll(extractBranches(chanceChild, newPath, newAccumulatedReward));
            } else if (child instanceof DecisionNode) {
                result.addAll(extractBranches(child, path, accumulatedReward));
            }
        }

        return result;
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
            double alpha = 1.;
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
            //double alpha = alphaDepth[(int)current.getDepth()];
            //alpha = 1./(10 * (dmax - current.getDepth()) - 3);
            double alpha = 1.;
            boolean progressiveWidening = children.size() <= FastMath.pow(current.getNumVisits(), alpha);
            //boolean progressiveWidening = FastMath.floor(FastMath.pow(current.getNumVisits(), alpha)) > FastMath.floor(FastMath.pow(current.getNumVisits() - 1, alpha));
            while(progressiveWidening && expandable) {

                // allow expansion of new node
                DecisionNode leaf = expand((DecisionNode) current, false);
                if (Objects.isNull(leaf)){
                    // objects not observable 
                    //expandable = false; // TODO: still backpropagate!
                    do{
                        // Travel down decision tree to search for expansion possibiliy
                        current = selectChildUCB(current);
                        children = current.getChildren();
                    } while (!current.getClass().getSimpleName().equals("DecisionNode"));

                    // If leaf is termination node, don't expand tree
                    if(current.getEpoch().compareTo(endCampaign) >= 0) {
                        backpropagate(current, null);
                        return false;
                    }
                    continue;

                } else if (leaf.getEpoch().compareTo(endCampaign) >= 0) {
                    // already reached end of campaign and acidently extended the tree beyond termination condition
                    if (((ChanceNode)leaf.getParent()).getMacro().getClass().getSimpleName().equals("TrackingObjective") || 
                        ((ChanceNode)leaf.getParent()).getMacro().getClass().getSimpleName().equals("CarTrackingObjective")) {     
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

            if(target.getClass().getSimpleName().equals("Car")) {
                Car targetCar = (Car) target;
                ObservedObject copy = 
                    new Car(targetCar.getIdentifier(), targetCar.getPosX(), targetCar.getPosY(), 
                            targetCar.getVelX(), targetCar.getVelY(), targetCar.getCov(), 
                            targetCar.getTime());
                restore.add(copy);
            } else {
                ObservedObject copy = new ObservedObject(target.getId(), target.getState(), 
                                                     target.getCovariance(), target.getEpoch(), 
                                                     target.getFrame());
                restore.add(copy);
            }
            
        }

        // Expand by Chance node first
        // Need to sample a new pair of macro and micro action
        double[] weights = new double[]{1., 1.};

        // Generate array filled with indexes representing the objective IDs
        int[] indexObjective = new int[weights.length];
        for (int i=0; i<weights.length; i++) {
            indexObjective[i] = i;
        }
        int indexSelectedObjective = 3;
            //WeightedRandomNumberPicker.pickNumber(indexObjective, weights);
        Objective objective;
        AngularDirection pointing = null;
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
                } else if(!searchAlreadyPerformed 
                            && endCampaign.durationFrom(leaf.getEpoch()) < TrackingObjective.allocation 
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
                //leaf.setEpochSensorPointing(leaf.getEpoch());
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
                // Macro action = track cars
                List<ObservedObject> ooiCar = new ArrayList<>(restore);

                for(Node sibling : leaf.getChildren()) {
                    ChanceNode chance = (ChanceNode)sibling;
                    if (chance.getMacro().getClass().getSimpleName().equals("CarTrackingObjective")) {
                        CarTrackingObjective track = (CarTrackingObjective)chance.getMacro();
                        char idAlreadyTracked = track.getLastUpdated();
                        int index = -1;
                        for(int i=0; i<ooiCar.size(); i++) {
                            if (ooiCar.get(i).getId() == idAlreadyTracked) {
                                index = i;
                                break;
                            }
                        }
                        if(index!=-1) {
                            ooiCar.remove(index);
                        }
                    }
                }
                objective = new CarTrackingObjective(ooiCar, this.startCampaign, 
                                                     this.endCampaign, this.predictedCars);
                pointing = objective.setMicroAction(leaf.getEpoch(), leaf.getSensorPointing());
                
                //throw new IllegalAccessError("Unknown objective.");
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
                                        this.initial.incrementIdCounter(), leaf.getDepth() + 0.5);      
        // Update 
        AbsoluteDate[] obsTimeInterval = expandedChance.getExecutionDuration();
        double executionDuration = obsTimeInterval[1].durationFrom(obsTimeInterval[0]);

        AngularDirection sensorPointing;
        String objectiveType = objective.getClass().getSimpleName();
        if (objectiveType.equals("SearchObjective")) {

            // Assign sensor pointing location
            List<AngularDirection> tasks = ((SearchObjective)expandedChance.getMacro()).getScheduleTopocentric();
            sensorPointing = tasks.get(tasks.size()-1);

        } else if(objectiveType.equals("TrackingObjective")) {

            // Assign sensor pointing location
            sensorPointing = expandedChance.getMicro();
        } else if(objectiveType.equals("CarTrackingObjective")) {

            // Assign sensor pointing location
            sensorPointing = expandedChance.getMicro();
        } else {
            throw new IllegalAccessError("Unknown objective.");
        }


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
            expandedDecision = new DecisionNode(0., 0, sensorPointing, propEpoch, environment,
                                                this.initial.incrementIdCounter(), 
                                                leaf.getDepth() + 1.0, leaf.getTimeSpentStripe());
        } else if (objective instanceof SearchObjective) {
            // searching objective has been selected TODO: hard copy of propagatedOutcome might be necessary
            // for now, only stripe scan is performed TODO: implement bullseye
            double taskT = expandedChance.getExecutionDuration()[1]
                            .durationFrom(expandedChance.getExecutionDuration()[0]);
            double updateTimeSpentSearch = leaf.getTimeSpentStripe() + taskT;
            List<Integer> propEnviroment = objective.propagateOutcome();
            //propEnviroment.set(0, (Integer)propEnviroment.get(0) + 1);
            PropoagatedEnvironment environment = 
                new PropoagatedEnvironment(leaf.getEnvironment().getStateTracking(), 
                                           propEnviroment);
            expandedDecision = new DecisionNode(0., 0, sensorPointing, propEpoch, environment,
                                                this.initial.incrementIdCounter(), 
                                                leaf.getDepth() + 1.0, updateTimeSpentSearch);

        } else if(objective instanceof CarTrackingObjective){
            List<ObservedObject> propEnviroment = new ArrayList<ObservedObject>();
            for(Car obj: (List<Car>)objective.propagateOutcome()) {
                Car copy = new Car(obj.getIdentifier(), obj.getPosX(), obj.getPosY(), 
                                   obj.getVelX(), obj.getVelY(), obj.getCov(), obj.getTime());
                propEnviroment.add(copy);
            }
            for(int parent=0; parent<leaf.getEnvironment().getStateTracking().size(); parent++) {
                char idParent = ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getIdentifier();
                boolean found = false;
                for(int child=0; child<propEnviroment.size(); child++) {
                    if (idParent == ((Car)propEnviroment.get(child)).getIdentifier()) {
                        found = true;
                    }
                }
                if(!found) {
                    Car notTargeted = new Car(idParent, ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getPosX(),
                                        ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getPosY(), 
                                        ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getVelX(), 
                                        ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getVelY(),
                                        ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getCov(),
                                        ((Car)leaf.getEnvironment().getStateTracking().get(parent)).getTime());
                    propEnviroment.add(notTargeted);
                }
            }
            PropoagatedEnvironment environment = 
                new PropoagatedEnvironment(propEnviroment, 
                                           leaf.getEnvironment().stateSearching);
            expandedDecision = new DecisionNode(0., 0, sensorPointing, propEpoch, environment,
                                                this.initial.incrementIdCounter(), 
                                                leaf.getDepth() + 1.0, leaf.getTimeSpentStripe());
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

        // restore environment
        if (orbitMode) {
            for(ObservedObject target : leaf.getEnvironment().getStateTracking()) {
                ObservedObject copy = new ObservedObject(target.getId(), target.getState(), 
                                                        target.getCovariance(), target.getEpoch(), 
                                                        target.getFrame());
                restore.add(copy);
            }
        } else {
            for(ObservedObject target : leaf.getEnvironment().getStateTracking()) {
                Car copy = new Car(((Car)target).getIdentifier(), ((Car)target).getStateArray(), 
                                   ((Car)target).getCov(), ((Car)target).getTime());
                restore.add(copy);
            }
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
    /* public Node backpropagateTimeUtility(Node leaf, Node last) {

        DecisionNode lastDecision;
        ChanceNode lastChance;
        Node parent;
        if (Objects.isNull(last)) {
            //No simulation was performed
            DecisionNode fakeRoot = 
                new DecisionNode(0., 0, null,
                                 leaf.getParent().getParent().getEpoch(), null,
                                 this.initial.incrementIdCounter(), 0., 0.);
            lastChance = 
                new ChanceNode(((ChanceNode)leaf.getParent()).getExecutionDuration(), leaf.getParent().getUtility(), 
                                leaf.getParent().getNumVisits(), ((ChanceNode)leaf.getParent()).getMacro(), 
                                ((ChanceNode)leaf.getParent()).getMicro(), fakeRoot,
                                this.initial.incrementIdCounter(), fakeRoot.getDepth() + 0.5);
            lastDecision = 
                new DecisionNode(leaf.getUtility(), leaf.getNumVisits(), ((DecisionNode)leaf).getSensorPointing(), 
                                leaf.getEpoch(), ((DecisionNode)leaf).getEnvironment(),
                                this.initial.incrementIdCounter(), fakeRoot.getDepth() + 1.0, 
                                ((DecisionNode)leaf).getTimeSpentStripe());
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
        
        double[] lastUtility = new double[2];
        double totalUtility = 0.;
       
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
    } */

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
            if (orbitMode) {
                for(ObservedObject obj : this.initial.getEnvironment().getStateTracking()) {
                    ObservedObject copy = new ObservedObject(obj.getId(), obj.getState(), 
                                                            obj.getCovariance(), obj.getEpoch(), 
                                                            obj.getFrame());
                    fakeObjs.add(copy);
                }
            } else {
                for(ObservedObject obj : this.initial.getEnvironment().getStateTracking()) {
                    Car copy = new Car(((Car)obj).getIdentifier(), ((Car)obj).getStateArray(), 
                                       ((Car)obj).getCov(), ((Car)obj).getTime());
                    fakeObjs.add(copy);
                }
            }

            List<Integer> fakeSearchTask = new ArrayList<Integer>();
            for(Integer task : this.initial.getEnvironment().getStateSearching()) {
                Integer copy = new Integer(task);
                fakeSearchTask.add(copy);
            }

            DecisionNode fakeRoot = 
                new DecisionNode(0., 0, null,  
                                 leaf.getParent().getParent().getEpoch(), 
                                 new PropoagatedEnvironment(fakeObjs, fakeSearchTask),
                                 this.initial.incrementIdCounter(), 0., 0.);
            lastChance = 
                new ChanceNode(((ChanceNode)leaf.getParent()).getExecutionDuration(), leaf.getParent().getUtility(), 
                                leaf.getParent().getNumVisits(), ((ChanceNode)leaf.getParent()).getMacro(), 
                                ((ChanceNode)leaf.getParent()).getMicro(), fakeRoot,
                                this.initial.incrementIdCounter(), fakeRoot.getDepth() + 0.5);
            lastDecision = 
                new DecisionNode(leaf.getUtility(), leaf.getNumVisits(), ((DecisionNode)leaf).getSensorPointing(),  
                                leaf.getEpoch(), ((DecisionNode)leaf).getEnvironment(),
                                this.initial.incrementIdCounter(), fakeRoot.getDepth() + 1.0, 
                                ((DecisionNode)leaf).getTimeSpentStripe());
            Node.setParent(lastDecision, lastChance);
        } else {
            lastDecision = (DecisionNode) last;
        }
        // Compute multi-objective utility value of leaf node
        computeUtilityVector(lastDecision, (DecisionNode)leaf);
        
        // add new utility vector to list of utilities
        //this.initial.addUtilityVec(leaf.getId(), utilityVec);
        
        //lastDecision.setUtility(nDom);

        /* Node current = leaf;

        while (!current.equals(this.initial)) {
            current.incrementNumVisits();
            // double updatedUtility = current.getUtility() + nDom;
            // current.setUtility(updatedUtility);
            double[] preUtilityVec = current.getUtilityVec();
            double[] postUtilityVec = new double[preUtilityVec.length];
            for(int i=0; i<preUtilityVec.length; i++) {

                // Update Value of the node
                postUtilityVec[i] = preUtilityVec[i] + (utilityVec[i] - preUtilityVec[i])
                                                        /current.getNumVisits();
                //postUtilityVec[i] = preUtilityVec[i] + utilityVec[i];
            }
            current.setUtilityVec(postUtilityVec);
            current = current.getParent();
        }

        this.initial.incrementNumVisits();
        // double updatedUtility = this.initial.getUtility() + nDom;
        // this.initial.setUtility(updatedUtility);
        double[] preUtilityVec = this.initial.getUtilityVec();
        double[] postUtilityVec = new double[preUtilityVec.length];
        for(int i=0; i<preUtilityVec.length; i++) {
            postUtilityVec[i] = preUtilityVec[i] + (utilityVec[i] - preUtilityVec[i]
                                                        /this.initial.getNumVisits());
        }        
        this.initial.setUtilityVec(postUtilityVec); */
        return this.initial;
    }

    /* private double computeRegret(DecisionNode lastDecision) {
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        char lastUpdated = ((CarTrackingObjective)parent.getMacro()).getLastUpdated();
        //List<Node> siblings = parent.getChildren();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Car sibling = (Car)objEnv;
            if (sibling.getIdentifier() != lastUpdated) {

                double timeUntilEnd = this.endCampaign.durationFrom(this.startCampaign);

                double simMeasPred = 
                    CarTrackingObjective.generateMeasurement(timeUntilEnd, sibling.getStateArray());
                Filter estLoss = new Filter();

                // Extract predicted covariance of sibling propagated to endCampaign 
                estLoss.run_ckf(sibling.getStateArray(), sibling.getCov(), sibling.getTime(), 
                                timeUntilEnd, simMeasPred);
                double[][] predCovSibling = estLoss.getCovPred();
                List<Car> predNoMeasSiblings = 
                    ((CarTrackingObjective)parent.getMacro()).getPredictedTargets();
                
                // Extract predicted covariance of sibling without measurement updates propagated 
                // to endCampaign 
                double[][] predCovNoMeasSibling = 
                    new double[predCovSibling.length][predCovSibling.length];
                for (Car same : predNoMeasSiblings) {
                    if (same.getIdentifier() == sibling.getIdentifier()) {
                        predCovNoMeasSibling = same.getCov();
                        break;
                    }
                }
                regret += CarTrackingObjective.computeTraceChange(predCovNoMeasSibling, 
                                                                  predCovSibling);
            }
        }
        return regret;
    } */


    /**
     * Return the utility vector. First entry contains search reward, the following all the rewards 
     * resulting from the tracking objective.
     * 
     * @param last          last simulated node.
     * @param leaf          last extisting node (without simulated nodes).
     * @return
     */
    private void computeUtilityVector(DecisionNode last, DecisionNode leaf) {

        // Compute tracking reward
        double[] trackReward = null;
        if (orbitMode) {
            trackReward = computeTrackReward(last);
        } else {
            double tCampaign = this.endCampaign.durationFrom(this.startCampaign);

            // Reward measured as regret
            CarTrackingObjective.computeTrackReward(last, leaf, this.initial, tCampaign, 
                                                    discount, this.sensor);
        }
        
        // Compute searching reward
/*         double searchReward = computeSearchReward(last, leaf); //TODO: function errornous because rSearch sometimes not zero
        if(searchReward!=0.) {
            System.out.println("Search reward erroneous");
        } */
/*         double searchReward = last.getTimeSpentStripe();

        double[] out = new double[trackReward.length + 1];
        out[0] = searchReward;
        for (int i=1; i<trackReward.length+1; i++) {
            out[i] = trackReward[i-1];
        }

        // Build up utility vector from macro action rewards                
        //return new double[]{searchReward, trackReward};
        return out; */
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
                //System.out.println("error");
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
    protected Node selectChildUCB(Node current) {

        if (current.getId()== 158) {
            System.out.println("check");
        }
        double maxUcb = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        double nP = current.getNumVisits();
    
        int maxUtility = Integer.MIN_VALUE;
        List<Long> optimalId = new ArrayList<Long>();
    
        double[] ucb = new double[current.getChildren().size()];
        double[] utilities = new double[current.getChildren().size()];
        
        if(current.getChildren().size() == 0) {
            return null;
        }
        List<double[]> utilityChildrenNorm = normaliseUtilityChildren(current.getChildren());
        
        // Prepare filter
        List<double[]> totalRtrackTotalRsearch = new ArrayList<double[]>();
        for(int i=0; i<current.getChildren().size(); i++) {

            double trackR = 0.;
            double[] normedR = utilityChildrenNorm.get(i);
            double searchT = normedR[0];
            Node child = current.getChildren().get(i);
            /* double timeTotal = 0.;
            if (child.getClass().getSimpleName().equals("DecisionNode")) {
                timeTotal = child.getEpoch().durationFrom(this.startCampaign);
            } else {
                timeTotal = child.getChildren().get(0).getEpoch().durationFrom(this.startCampaign);
            } */
           double timeTotal = this.endCampaign.durationFrom(this.startCampaign);
            double searchR = FastMath.abs(timeTotal * this.userSearchTrequested - searchT);
            searchR = 0.;
            for(int k=1; k<normedR.length; k++){
                trackR = normedR[k] + trackR;
            }
            totalRtrackTotalRsearch.add(new double[]{searchR, trackR}); // scalarised tracking reward
            //totalRtrackTotalRsearch.add(normedR);
        }

        // Filter
        for (int i=0; i<current.getChildren().size(); i++){
            double[] removedUtility = totalRtrackTotalRsearch.remove(0);
    
            OptimisingVector opt = new OptimisingVector(totalRtrackTotalRsearch, removedUtility.length - 1);
            List<double[]> domVecs = 
                opt.getDominatingVecs(removedUtility, 
                                      new boolean[]{false, true}, 
                                      0);
            int utility = - domVecs.size();
            if(utility>maxUtility) {
                maxUtility = utility;
                optimalId.clear();
                optimalId.add(current.getChildren().get(i).getId());
            } else if (utility==maxUtility){
                optimalId.add(current.getChildren().get(i).getId());
            } 
            
            utilities[i] = current.getChildren().get(i).getUtility() + utility;
            totalRtrackTotalRsearch.add(removedUtility);
        }
    
        // Compute ucb values
        for (int i=0; i<utilities.length; i++){
 
            current.getChildren().get(i).setUtility(utilities[i]);
            double n = current.getChildren().get(i).getNumVisits();
            ucb[i] = utilities[i] + C * FastMath.sqrt(FastMath.log(nP)/n);
        }
    
        // search for child that maximises ucb
        Random rand = new Random();
        for(int i=0; i<ucb.length; i++) { 
            if(ucb[i]>maxUcb) {
                maxUcb = ucb[i];
            } 
        }
        List<Node> bestCandidates = new ArrayList<>();
        for (int i=0; i<ucb.length; i++) {
            if (ucb[i] == maxUcb) {
                bestCandidates.add(current.getChildren().get(i));
            }
        }
        // Pick one randomly
        potentiallySelected = bestCandidates.get(rand.nextInt(bestCandidates.size()));
        return potentiallySelected;
    }

    private static List<double[]> normaliseUtilityChildren(List<Node> children) {
        List<double[]> utilityChildrenNorm = new ArrayList<double[]>();
        for (Node child : children) {
            double[] utilityNorm = new double[child.getUtilityVec().length];
            for(int i=0; i<utilityNorm.length; i++) {   
                //utilityNorm[i] = child.getUtilityVec()[i] / child.getNumVisits();
                utilityNorm[i] = child.getUtilityVec()[i];
            }
            utilityChildrenNorm.add(utilityNorm); 
        }
        return utilityChildrenNorm;
    }


    protected static Node selectChildRobustMax(Node current) {
        double robustMax = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        
        for (Node child : current.getChildren()){
            double n = child.getNumVisits();
            double v = child.getUtility();
            double sum = v + n;

            // search for child that maximises the sum of visits and values
            if (sum>robustMax) {
                potentiallySelected = child;
                robustMax = sum;
            } // TODO: need to add removed utility to utilityChildrenNorm
        }
        return potentiallySelected;
    }

    protected static Node selectChildMaxUtility(Node current) {
        double maxUtility = Double.NEGATIVE_INFINITY;
        Node potentiallySelected = null;
        
        for (Node child : current.getChildren()){
            double v = child.getUtility();

            // search for child that maximises the sum of visits and values
            if (v>maxUtility) {
                potentiallySelected = child;
                maxUtility = v;
            } // TODO: need to add removed utility to utilityChildrenNorm
        }
        return potentiallySelected;
    }
}
