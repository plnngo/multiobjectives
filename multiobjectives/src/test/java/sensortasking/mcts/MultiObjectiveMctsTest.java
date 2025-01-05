package sensortasking.mcts;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.BlockRealMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

import com.opencsv.CSVWriter;

import tools.OptimisingVector;

public class MultiObjectiveMctsTest {

    /** Tree structure stored in a root node. */
    Node root;

    Frame j2000 = FramesFactory.getEME2000(); 

    @Before
    public void init() {
        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Build up test decision tree
        /* root = new DecisionNode(28, 4, null, null, null, new AbsoluteDate(), null, 0);
        root.setId(0);
        
        ChanceNode child1 = new ChanceNode(null, 17, 2, null, null, root, 
                                           ((DecisionNode)root).incrementIdCounter());
        ChanceNode child2 = new ChanceNode(null, 8, 2, null, null, root, 
                                           ((DecisionNode)root).incrementIdCounter());

        root.setChild(child1);
        root.setChild(child2);

        DecisionNode grandchild1 = new DecisionNode(10, 1, null, null, null, new AbsoluteDate(), null,
                                                    ((DecisionNode)root).incrementIdCounter());
        DecisionNode grandchild2 = new DecisionNode(7, 1, null, null, null, new AbsoluteDate(), null,
                                                    ((DecisionNode)root).incrementIdCounter());
        DecisionNode grandchild3 = new DecisionNode(3, 1, null, null, null, new AbsoluteDate(), null,
                                                    ((DecisionNode)root).incrementIdCounter());
        DecisionNode grandchild4 = new DecisionNode(5, 1, null, null, null, new AbsoluteDate(), null,
                                                    ((DecisionNode)root).incrementIdCounter());
        child1.setChild(grandchild1);
        child1.setChild(grandchild2);
        child2.setChild(grandchild3);
        child2.setChild(grandchild4);

        ChanceNode ggchild1 = new ChanceNode(null, 10, 1, null, null, grandchild1,
                                             ((DecisionNode)root).incrementIdCounter());
        ChanceNode ggchild2 = new ChanceNode(null, 7, 1, null, null, grandchild2,
                                             ((DecisionNode)root).incrementIdCounter());

        grandchild1.setChild(ggchild1);
        grandchild2.setChild(ggchild2);

        DecisionNode gggchild1 = new DecisionNode(10, 1, null, null, null, new AbsoluteDate(), null,
                                                  ((DecisionNode)root).incrementIdCounter());
        DecisionNode gggchild2 = new DecisionNode(7, 1, null, null, null, new AbsoluteDate(), null,
                                                  ((DecisionNode)root).incrementIdCounter());

        ggchild1.setChild(gggchild1);
        ggchild2.setChild(gggchild2); */
    }   

    @Test
    public void testSelectChild() {

        Node parent = new Node();
        parent.setUtility(50);
        parent.setNumVisits(9);

        int[] numVisits = new int[]{3, 6, 1, 7, 8, 3, 2, 1, 1, 9};
        for(int i=0; i<numVisits.length; i++){
            double utility = i * 10;
            Node child = new Node();
            child.setUtility(utility);
            child.setNumVisits(numVisits[i]);

            parent.setChild(child);
        }
        Node actuallySelected = MultiObjectiveMcts.selectChildUCB(parent);

        // Last child is expected to reveal largest UCB because of its large utility value
        Node expectedlySelected = parent.getChildren().get(numVisits.length - 1);
        Assert.assertEquals(expectedlySelected, actuallySelected);
    }

/*     @Test
    public void testSelect() {
        // Compare
        List<Node> actual = MultiObjectiveMcts.select(root);
        long[] expectedIds = new long[]{0, 1, 3, 7, 9};
        for(int i=0; i<expectedIds.length; i++) {
            Assert.assertEquals(expectedIds[i], actual.get(i).getId());
        }
    } */

/*    @Test
     public void testExpandFromChanceNode(){
        double tObs = 480. * 60.;
        double[] initialWeight = new double[]{1., 0.};
        double[] initialTimeResources = new double[]{tObs * initialWeight[0], tObs * initialWeight[1]};
        DecisionNode root = new DecisionNode(0., 0, null, initialWeight, initialTimeResources,
                                             new AbsoluteDate(), new ArrayList<ObservedObject>());

        double executionDuration = 5. * 60.;
        Objective search = new SearchObjective();
        AngularDirection pointing = search.setMicroAction(new AbsoluteDate());
        ChanceNode leaf = new ChanceNode(executionDuration, 0., 0, search, pointing, root);
        Node actual = MultiObjectiveMcts.expand(leaf);
        
        // Compare
        DecisionNode castedActual = (DecisionNode) actual;
        AngularDirection actualPointing = castedActual.getSensorPointing();
        ObservedObject detection = castedActual.getPropEnvironment().get(0);
        //RealMatrix expectedCov = new DiagonalMatrix(new double[]{10., 10., 10., 100., 100., 100.});

        Assert.assertEquals("DecisionNode", actual.getClass().getSimpleName());
        Assert.assertEquals(5.*60., actual.getEpoch().durationFrom(leaf.getEpoch()), 1E-16);
        Assert.assertEquals(AngleType.AZEL, actualPointing.getAngleType());
        Assert.assertEquals(FastMath.toRadians(30.), actualPointing.getAngle1(), 1E-16);
        Assert.assertEquals(FastMath.toRadians(50.), actualPointing.getAngle2(), 1E-16);
        Assert.assertEquals(new Vector3D(100., 200., 300.), 
                            detection.getState().getPositionVector());
        //Assert.assertEquals(expectedCov, detection.getCovariance().getCovarianceMatrix());
    } */

/*     @Test
    public void testExpandFromDecisionNode(){
        double tObs = 480. * 60.;
        double[] initialWeight = new double[]{0., 1.};
        double[] initialTimeResources = new double[]{tObs * initialWeight[0], tObs * initialWeight[1]};
        DecisionNode leaf = new DecisionNode(0., 0, null, initialWeight, initialTimeResources,
                                             new AbsoluteDate(), new ArrayList<ObservedObject>());
        Node actual = MultiObjectiveMcts.expand(leaf);

        // Compare
        Assert.assertEquals("ChanceNode", actual.getClass().getSimpleName());
        ChanceNode castedActual = (ChanceNode) actual;
        AngularDirection actualPointing = castedActual.getMicro();
        Assert.assertEquals(leaf.getEpoch(), castedActual.getEpoch());
        Assert.assertEquals("TrackingObjective", 
                                     castedActual.getMacro().getClass().getSimpleName());
        Assert.assertEquals(60.*5., castedActual.getExecutionDuration(), 1E-16);
        Assert.assertEquals(AngleType.AZEL, actualPointing.getAngleType());
        Assert.assertEquals(FastMath.toRadians(88.), actualPointing.getAngle1(), 1E-16);
        Assert.assertEquals(FastMath.toRadians(30.), actualPointing.getAngle2(), 1E-16);
    } */

/*     @Test
    public void testSimulate() {
        double tObs = 480. * 60.;
        double[] initialWeight = new double[]{0.5, 0.5};
        double[] initialTimeResources = new double[]{tObs * initialWeight[0], tObs * initialWeight[1]};
        DecisionNode leaf = new DecisionNode(0., 0, null, initialWeight, initialTimeResources,
                                             new AbsoluteDate(), new ArrayList<ObservedObject>());
        List<Node> actual = MultiObjectiveMcts.simulate(leaf, leaf.getEpoch().shiftedBy(15.*60.));
        Assert.assertEquals(9, actual.size());
    } */

/*     @Test
    public void testBackpropagate() {

        // Set up
        long[] expectedIds = new long[]{0, 1, 3, 7, 9};
        List<Node> selected = MultiObjectiveMcts.select(root);
        MultiObjectiveMcts mcts = new MultiObjectiveMcts(root, null, null, null);

        // Add fakely simulated nodes
        DecisionNode termination = new DecisionNode(1, 1, null, null, null, null, null);

        mcts.backpropagate(selected, termination);
        Node updatedRoot = mcts.getInitial();
        List<Node> updatedEpisode = MultiObjectiveMcts.select(updatedRoot);

        // Updated tree should result in the same nodes selected using the UCB condition
        for(int i=0; i<expectedIds.length; i++) {
            Assert.assertEquals(expectedIds[i], updatedEpisode.get(i).getId());
        }

        // Compare root node
        Assert.assertEquals(5, updatedEpisode.get(0).getNumVisits());
        Assert.assertEquals(29, updatedEpisode.get(0).getUtility(),1e-16);

        // Compare next descendant
        Assert.assertEquals(3, updatedEpisode.get(1).getNumVisits());
        Assert.assertEquals(18, updatedEpisode.get(1).getUtility(),1e-16);

        // Compare the rest of the episode
        for(int i=2; i<updatedEpisode.size(); i++) {
            Assert.assertEquals(2, updatedEpisode.get(i).getNumVisits());
            Assert.assertEquals(11, updatedEpisode.get(i).getUtility(),1e-16);
        }
    } */

    @Test
    public void testUtilityPerformance() {
      
        // Epoch
        AbsoluteDate current = (new AbsoluteDate(2024, 8, 2, 3, 24, 0., TimeScalesFactory.getUTC())).shiftedBy(4000.);
        AbsoluteDate endCampaign = current.shiftedBy(10.* 60.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");
        Transform horizonToEci = topohorizon.getTransformTo(j2000, current);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(current, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        // Retrieve object of interest that shall be tracked
        List<ObservedObject> ooiAll = setListOOI(current);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(ooiAll.get(0));

        // Set environment storing the state of the tasking outputs
        List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
        stripeBullseyeCompleted.add(0); // Stripe scan
        stripeBullseyeCompleted.add(0); // Bullseye scan

        PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);

        // Initialise root node
        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));
        double initUtility = 1.;
        int numVisits = 1;
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC);
        double[] initWeights = new double[]{1.0, 0.0};
        double[] initTimeResources = 
            new double[]{initWeights[0] * endCampaign.durationFrom(current), 
                         initWeights[1] * endCampaign.durationFrom(current)};
        Node root = new DecisionNode(initUtility, numVisits, initPointing, initWeights, 
                                     initTimeResources, current, enviro, 0);
        MultiObjectiveMcts mctsTracking = 
            new MultiObjectiveMcts(root, objectives, current, endCampaign, "TDRS Station", ooi, 
                                   new ArrayList<ObservedObject>(), sensor);
        List<Node> strategy = mctsTracking.run(10);
        for(Node selected : strategy) {
            if(selected.getClass().getSimpleName().equals("ChanceNode")) {

                //System.out.println("Next node");
                ChanceNode chance = (ChanceNode)selected;
                
                if (chance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                    System.out.println("Search from " + chance.getExecutionDuration()[0] + " to " + chance.getExecutionDuration()[1]);
                } else {
                    System.out.println("Track from " + chance.getExecutionDuration()[0] + " to " + chance.getExecutionDuration()[1]);
                }
            }
        }
        double timeLeftSearch = ((DecisionNode)strategy.get(strategy.size()-1)).getTimeResources()[0];
        double timeLeftTrack = ((DecisionNode)strategy.get(strategy.size()-1)).getTimeResources()[1];
        System.out.println("Time left search: " + timeLeftSearch + " and time left track: " + timeLeftTrack);
    }
    @Test
    public void testMctsSearchAndTrackAsMC() {
        long start1 = System.nanoTime();
      
        // Epoch
        //AbsoluteDate current = (new AbsoluteDate(2024, 8, 2, 3, 24, 0., TimeScalesFactory.getUTC())).shiftedBy(4000.);
        AbsoluteDate current = new AbsoluteDate(2024, 8, 2, 3, 24, 0., TimeScalesFactory.getUTC());

        AbsoluteDate endCampaign = current.shiftedBy(60.* 60. * 2);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");
        Transform horizonToEci = topohorizon.getTransformTo(j2000, current);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(current, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        /* // Station in ECEF
        Transform horizonToEcef = topohorizon.getTransformTo(ecef, current);  // date has to be the measurement epoch
        Vector3D coordinatesStationEcef = horizonToEcef.transformPosition(Vector3D.ZERO);
        System.out.println(coordinatesStationEcef.getX() * 1e-3 + " " + coordinatesStationEcef.getY() * 1e-3 + " " + coordinatesStationEcef.getZ() * 1e-3); */

        // Retrieve object of interest that shall be tracked
        List<ObservedObject> ooi = setListOOI(current);
        List<ObservedObject> initialOoi = new ArrayList<ObservedObject>(ooi);

        // Set environment storing the state of the tasking outputs
        List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
        stripeBullseyeCompleted.add(0); // Stripe scan
        stripeBullseyeCompleted.add(0); // Bullseye scan

        PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);

        Transform eciToHorizon = j2000.getTransformTo(topohorizon, current);
        Vector3D tdrs6Horizon = eciToHorizon.transformPosition(ooi.get(1).getState().getPositionVector());
        System.out.println("Elevation: " + FastMath.toDegrees(tdrs6Horizon.getDelta()));

        // Initialise root node
        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));
        double initUtility = 1.;
        int numVisits = 1;
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC);
        double[] initWeights = new double[]{0.5, 0.5};
        double[] initTimeResources = 
            new double[]{initWeights[0] * endCampaign.durationFrom(current), 
                         initWeights[1] * endCampaign.durationFrom(current)};
        Node root = new DecisionNode(initUtility, numVisits, initPointing, initWeights, 
                                     initTimeResources, current, enviro, 0)
                        .setWeightsSearchingTask(new double[]{1.0, 0.});
        MultiObjectiveMcts mctsTracking = 
            new MultiObjectiveMcts(root, objectives, current, endCampaign, "TDRS Station", ooi, 
                                   new ArrayList<ObservedObject>(), sensor);
        List<Node> strategy = mctsTracking.run(70000000);
        
        //performIODsearch(strategy, current, fov, topohorizon, initialOoi);

        long end1 = System.nanoTime();      
        System.out.println("Elapsed Time in nano seconds: "+ (end1-start1));
    }

/*     private void performIODsearch(List<Node> strategy, AbsoluteDate current, Fov fov, 
                                  TopocentricFrame topohorizon, List<ObservedObject> ooi) {

        // Extract observable objects
        List<TLE> observables = getGEOSat(current);

        // Initialise Kalman Filter
        double[] xbar0 = new double[]{0., 0., 0., 0., 0., 0.};
        int counter = 0;
        
        // Create hard copy
        List<ObservedObject> ooiInitial = new ArrayList<ObservedObject>();
        for(ObservedObject copy : ooi) {
            ooiInitial.add(new ObservedObject(copy.getId(), copy.getState(), copy.getCovariance(), 
                                       copy.getEpoch(), copy.getFrame()));
        }

        // Perform tasks
        //List<AngularDirection> actualMeas = new ArrayList<AngularDirection>();
        Map<Integer, List<AngularDirection>> measMap = new HashMap<Integer, List<AngularDirection>>();
        File file = new File("PlotTrackingData.csv"); 
        int countStripes = 0;

        try { 
            // create FileWriter object with file as parameter 
            FileWriter outputfile = new FileWriter(file); 
    
            // create CSVWriter object filewriter object as parameter 
            CSVWriter writer = new CSVWriter(outputfile); 
    
            // adding header to csv 
            String[] header = { "Satellite number", "Time",
                                "PosX error", "PosY error", "PosZ error", 
                                "VelX error", "VelY error", "VelZ error", 
                                "CovX", "CovY", "CovZ", "CovXdot", "CovYdot", "CovZdot",
                                "Ra Residual", "Dec Residual",
                                "PosX Truth", "PosY Truth", "PosZ Truth",
                                "VelX Truth", "VelY Truth", "VelZ Truth",
                                "CovXY", "CovXZ", "CovYZ" }; 
            writer.writeNext(header); 

            for (Node selected : strategy) {
                if(Objects.isNull(selected)) {
                    continue;
                }
                if(selected.getClass().getSimpleName().equals("ChanceNode")) {

                    //System.out.println("Next node");
                    ChanceNode chance = (ChanceNode)selected;
                    
                    if (chance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {
                        countStripes++;

                        List<AngularDirection> tasks = ((SearchObjective)chance.getMacro()).getScheduleGeocentric();
                        for(AngularDirection task: tasks) {
                            AbsoluteDate epoch = task.getDate();
                            double[] raRange = new double[]{task.getAngle1() - fov.getWidth()/2, 
                                                            task.getAngle1() + fov.getWidth()/2};
                            double[] decRange = new double[]{task.getAngle2() - fov.getHeight()/2,
                                                            task.getAngle2() + fov.getHeight()/2};
                
                            for(TLE candidate : observables) {
                                TLEPropagator prop = TLEPropagator.selectExtrapolator(candidate);
                                Vector3D propPos = prop.propagate(epoch).getPVCoordinates(j2000).getPosition();
                                AngularDirection anglePos = 
                                    new AngularDirection(j2000, 
                                                        new double[]{propPos.getAlpha(), propPos.getDelta()}, 
                                                        AngleType.RADEC);
                                boolean inDecField = checkInAngularRange(anglePos, raRange, decRange);
                
                                // Extract measurement if object is in FOV
                                if(inDecField) {
                                    // Transform into topocentric horizon frame
                                    anglePos = anglePos.transformReference(topohorizon, epoch, AngleType.AZEL);
                                    anglePos.setDate(epoch);
                                    anglePos.setName(Integer.toString(candidate.getSatelliteNumber()));
                                    System.out.println("Detect: " + candidate.getSatelliteNumber() + " at " + epoch);
                                    List<AngularDirection> registered = measMap.get(candidate.getSatelliteNumber());
                                    if(Objects.isNull(registered)) {
                                        registered = new ArrayList<AngularDirection>();
                                    }
                                    registered.add(anglePos);
                                    measMap.put(candidate.getSatelliteNumber(), registered);
                                }
                            }
                        }
                    } else {
                        TrackingObjective objective = (TrackingObjective)chance.getMacro();
                        long lastUpdated = objective.getLastUpdated();
                        //AbsoluteDate epoch = chance.getEpoch();
                        AngularDirection rawMeas = chance.getMicro();
                        Random r = new java.util.Random();
                        double mean = 0.;
                        double variance = FastMath.pow(1./206265., 2);
                        double noiseRa = r.nextGaussian() * FastMath.sqrt(variance) + mean;
                        double noiseDec = r.nextGaussian() * FastMath.sqrt(variance) + mean;
                        double[] noisyAngles = new double[]{rawMeas.getAngle1() + noiseRa, 
                                                            rawMeas.getAngle2() + noiseDec};

                        AngularDirection measWithNoise = new AngularDirection(rawMeas.getFrame(), 
                                                                            noisyAngles, 
                                                                            rawMeas.getAngleType());
                        measWithNoise.setDate(rawMeas.getDate());
                        //measWithNoise = rawMeas;

                        // Propagate target to measurement epoch
                        ObservedObject candidate = null;
                        for(ObservedObject target : ooi) {
                            if(target.getId() == lastUpdated) {
                                candidate = target;         // TODO: check if frame is EME2000
                                break;
                            }
                        }
                        Vector3D pos = candidate.getState().getPositionVector();
                        Vector3D vel = candidate.getState().getVelocityVector();
                        PVCoordinates pv = new PVCoordinates(pos, vel);
                        Orbit initialOrbit = new CartesianOrbit(pv, candidate.getFrame(), 
                                                candidate.getEpoch(), Constants.WGS84_EARTH_MU);
                        KeplerianPropagator kepPropo = new KeplerianPropagator(initialOrbit);
                        RealMatrix R = 
                            MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow( 1./206265, 2), 
                                                                    FastMath.pow( 1./206265, 2)});
                        
                        double[] angleRes = new double[2];
                        ObservedObject[] predAndCorr = 
                            TrackingObjective.estimateStateWithOwnExtendedKalman(kepPropo, 
                                                                                 measWithNoise, R, 
                                                                                 candidate, 
                                                                                 angleRes);
                        for(int i=0; i<ooi.size(); i++) {
                            if(ooi.get(i).getId() == candidate.getId()) {

                                // Compute state error
                                Vector3D posOD = predAndCorr[1].getState().getPositionVector();
                                Vector3D velOD = predAndCorr[1].getState().getVelocityVector();
                                Vector3D posError = Vector3D.ZERO;
                                Vector3D velError = Vector3D.ZERO;
                                Vector3D posTruth = Vector3D.ZERO;
                                Vector3D velTruth = Vector3D.ZERO;

                                for(ObservedObject target : ooiInitial) {
                                    if(target.getId() == candidate.getId()) {
                                        Vector3D posInitial = target.getState().getPositionVector();
                                        Vector3D velInitial = target.getState().getVelocityVector();
                                        PVCoordinates pvInitial = new PVCoordinates(posInitial, velInitial);
                                        Orbit initial = new CartesianOrbit(pvInitial, target.getFrame(), 
                                                                        target.getEpoch(), Constants.WGS84_EARTH_MU);
                                        KeplerianPropagator prop = new KeplerianPropagator(initial);
                                        SpacecraftState truthState = prop.propagate(predAndCorr[1].getEpoch());
                                        posTruth = truthState.getPVCoordinates().getPosition();
                                        velTruth = truthState.getPVCoordinates().getVelocity();
                                        System.out.println(posTruth.getX() * 1e-3 + " " + posTruth.getY() * 1e-3 + " " + posTruth.getZ() * 1e-3);
                                        System.out.println(velTruth.getX() * 1e-3 + " " + velTruth.getY() * 1e-3 + " " + velTruth.getZ() * 1e-3);


                                        posError = posOD.subtract(posTruth);
                                        velError = velOD.subtract(velTruth);
                                       
                                        break;
                                    }
                                }

                                // Extract standard deviation in pos and vel in ECI
                                double[] sigma = new double[6];
                                CartesianCovariance cov = predAndCorr[1].getCovariance();
                                RealMatrix covMatrix = cov.getCovarianceMatrix();
                                //App.printCovariance(covMatrix);
                                for(int j=0; j<sigma.length; j++) {
                                    sigma[j] = FastMath.sqrt(covMatrix.getEntry(j, j));
                                }
                                double[] sigma2Side = new double[3];
                                sigma2Side[0] = covMatrix.getEntry(1, 0);
                                sigma2Side[1] = covMatrix.getEntry(2, 0);
                                sigma2Side[2] = covMatrix.getEntry(2, 1);

                                // Print file
                                AngularDirection residuals = rawMeas.substract(measWithNoise);
                                String[] data = { Long.toString(candidate.getId()), Double.toString(predAndCorr[1].getEpoch().durationFrom(ooiInitial.get(0).getEpoch())),
                                                  Double.toString(posError.getX()), Double.toString(posError.getY()), Double.toString(posError.getZ()),
                                                  Double.toString(velError.getX()), Double.toString(velError.getY()), Double.toString(velError.getZ()),
                                                  Double.toString(sigma[0]), Double.toString(sigma[1]), Double.toString(sigma[2]),
                                                  Double.toString(sigma[3]), Double.toString(sigma[4]), Double.toString(sigma[5]),
                                                  Double.toString(angleRes[0]), Double.toString(angleRes[1]),
                                                  //Double.toString(residuals.getAngles()[0]), Double.toString(residuals.getAngle2()),
                                                  Double.toString(posTruth.getX()), Double.toString(posTruth.getY()), Double.toString(posTruth.getZ()),
                                                  Double.toString(velTruth.getX()), Double.toString(velTruth.getY()), Double.toString(velTruth.getZ()),
                                                  Double.toString(sigma2Side[0]), Double.toString(sigma2Side[1]), Double.toString(sigma2Side[2])}; 
                                writer.writeNext(data); 

                                // Compute vector norm 
                                double vecnorm = 0.;
                                for (double entry : xbar0) {
                                    vecnorm += FastMath.pow(entry, 2);
                                }
                                vecnorm = FastMath.sqrt(vecnorm);

                                // State reference update
                                ooi.get(i).setCovariance(cov);
                                ooi.get(i).setState(predAndCorr[1].getState());
                                ooi.get(i).setEpoch(predAndCorr[1].getEpoch());
                                ooi.get(i).setFrame(predAndCorr[1].getFrame());
                                xbar0 = new double[]{0., 0., 0., 0., 0., 0.};
                                counter = 0;

                                break;

                            }
                        }
                    }
                }
            }
            // closing writer connection 
            writer.close(); 
        } 
        catch (IOException e) { 
            // TODO Auto-generated catch block 
            e.printStackTrace(); 
        } 
        double timeLeftSearch = ((DecisionNode)strategy.get(strategy.size()-1)).getTimeResources()[0];
        double timeLeftTrack = ((DecisionNode)strategy.get(strategy.size()-1)).getTimeResources()[1];
        System.out.println("Time left search: " + timeLeftSearch + " and time left track: " + timeLeftTrack);

        System.out.println("Number of objects detected " + measMap.size());
        //IodGauss iod = new IodGauss(TLEConstants.MU);
        IodGooding iod = new IodGooding(TLEConstants.MU);
        GroundStation stationOrekit = new GroundStation(topohorizon);
        List<Orbit> iodOrbits = new ArrayList<Orbit>();
        System.out.println("Number of stripe scans: " + countStripes);
        for (Map.Entry<Integer, List<AngularDirection>> entry : measMap.entrySet()) {
            List<AngularDirection> meas = entry.getValue();
            if(meas.size()<3){
                System.out.println(meas.size() + "are enough data for " + entry.getKey() + " for IOD.");
                continue;
            } 
            List<AngularAzEl> measOrekit = new ArrayList<AngularAzEl>();
            for (int i=0; i<3; i++) {
                measOrekit.add(new AngularAzEl(stationOrekit, meas.get(i).getDate(), new double[]{meas.get(i).getAngle1(), 
                                meas.get(i).getAngle2()}, new double[]{FastMath.pow(1./206265., 2),
                                FastMath.pow(1./206265., 2)}, new double[]{1., 1.}, 
                                new ObservableSatellite(0)));
            }

            Orbit iodNew = iod.estimate(j2000, measOrekit.get(0), 
                                        measOrekit.get(1), measOrekit.get(2), 
                                        TLEConstants.EARTH_RADIUS * 1e3, 
                                        TLEConstants.EARTH_RADIUS * 1e3);
            iodOrbits.add(iodNew);
        }
        System.out.println(iodOrbits.size());
    } */

    @Test
    public void testEstimateStateWithOwnKalman(){
        // Frame
        Frame eci = FramesFactory.getEME2000();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        final AbsoluteDate target = new AbsoluteDate("2000-01-01T15:50:04.36035985407872Z", TimeScalesFactory.getUTC());


        // Definition of initial conditions with position and velocity
        //------------------------------------------------------------
        Vector3D position = new Vector3D(7.0e6, 1.0e6, 4.0e6);
        Vector3D velocity = new Vector3D(-500.0, 8000.0, 1000.0);
        PVCoordinates pvInit = new PVCoordinates(position, velocity);
        double mu = 3.9860047e14;
        System.out.println("----- Initial condition -----");
        System.out.println("MU [m^3/s^-2]: \t" + mu);
        AbsoluteDate initDate = AbsoluteDate.J2000_EPOCH.shiftedBy(584.);
        Orbit initialOrbit = new CartesianOrbit(pvInit, eci, initDate, mu);
        System.out.println("Initial date: \t" + initDate);

        // Extrapolator definition
        // -----------------------
        KeplerianPropagator extrapolator = new KeplerianPropagator(initialOrbit);
        System.out.println("Propagator: \t" + extrapolator.toString());
        System.out.println("Frame: \t" + eci);
        System.out.println("Position [m]: \t" + position);
        System.out.println("Velocity [m/s]: \t" + velocity);
        System.out.println("Initial covariance: ");
        final SpacecraftState initialState = extrapolator.getInitialState();

        // Initial covariance
        RealMatrix covInitMatrix = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                              0.1, 0.1, 0.1});
/*         final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = 
            extrapolator.setupMatricesComputation(stmAdditionalName, null, null); */
        SpacecraftState finalOrbit = extrapolator.propagate(target);
        // Predicted state
        Vector3D predictedPos = finalOrbit.getPVCoordinates().getPosition();
        Vector3D predictedVel = finalOrbit.getPVCoordinates().getVelocity();

        // Set up station
        Transform eciToEcef = eci.getTransformTo(ecef, target);       
        PVCoordinates pvEcef = 
            eciToEcef.transformPVCoordinates(new PVCoordinates(predictedPos, predictedVel));
        double lon = pvEcef.getPosition().getAlpha();
        double lat = pvEcef.getPosition().getDelta();

        GeodeticPoint station = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "New Station");
        Transform horizonToEci = topoHorizon.getTransformTo(eci, target);  
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        double readout = 7.;
        double exposure = 8.;
        double settling = 30.;
        double cutOff = FastMath.toRadians(5.);
        double slewT = 9.;
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = fov.getHeight()/slewT;
        Sensor sensor = new Sensor("Sensor", fov, station, exposure, readout, slewVel, settling, cutOff);

        Transform eciToTopo = new Transform(target, coordinatesStationEci.negate());
/*         Frame topoCentric = new Frame(eci, eciToTopo, "Topocentric", true);

        // Generate real measurement
        PVCoordinates pvTopo = 
            eciToTopo.transformPVCoordinates(new PVCoordinates(predictedPos, predictedVel));
        double ra = pvTopo.getPosition().getAlpha();
        double dec = pvTopo.getPosition().getDelta(); */
        //AngularDirection realRaDec = new AngularDirection(topoCentric, new double[]{ra, dec}, AngleType.RADEC);

        RealMatrix R = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow(1./206265, 2), 
                                                              FastMath.pow(1./206265, 2)});

        StateVector state = ObservedObject.spacecraftStateToStateVector(initialState, j2000);
        StateCovariance covEci = new StateCovariance(covInitMatrix, initDate, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance stateCov =
            ObservedObject.stateCovToCartesianCov(finalOrbit.getOrbit(), covEci, j2000); 
        ObservedObject candidate = new ObservedObject(123, state, stateCov, initDate, j2000);
        TrackingObjective.estimateStateWithOwnExtendedKalman(extrapolator, target, R, 
                                                             candidate, new double[2], sensor);
    }

    @Test
    public void computeMahalanobis() {
        // Load TLE and covariance
        TLE wgsf1 = new TLE("1 32258U 07046A   24214.22740957 -.00000276  00000-0  00000-0 0  9992",
                            "2 32258   0.0871  93.8244 0000689  26.5645 229.2946  1.00273367 45472"); 
        TLEPropagator tleProp = TLEPropagator.selectExtrapolator(wgsf1);

        List<double[]> raw = new ArrayList<double[]>();
        raw.add(new double[]{0.009857869368511499});
        raw.add(new double[]{4.6206115891553294E-7, 0.009855922757369481});
        raw.add(new double[]{-3.581241368850455E-9, -2.035932706968922E-9, 0.009857034792855091});
        raw.add(new double[]{4.3338320997620344E-6, -2.752930753343345E-8, -2.2699783010120159E-10, 1.676123458384334E-8});
        raw.add(new double[]{-2.8349102557690033E-8, 4.302047005245893E-6, 1.2354203033401438E-10, 6.481271106982781E-10, 1.787875164302044E-8});
        raw.add(new double[]{-2.2702775480075429E-10, 1.2000001614458115E-10, 4.281458191767511E-6, 6.381142028875982E-12, -2.8215995574484546E-12, 1.8234721264595634E-8});
        RealMatrix covWgsf1 = setCovariance(raw);

        // transform from EME2000 to TEME
        final Frame j2000 = FramesFactory.getEME2000();
        final Frame teme  = FramesFactory.getTEME();
        final double[][] jacJ2000ToTEME = new double[6][6];
        j2000.getTransformTo(teme, wgsf1.getDate()).getJacobian(CartesianDerivativesFilter.USE_PV, jacJ2000ToTEME);
                
        // Covariance transformation, using Hipparchus RealMatrix class to perform the multiplication
        final RealMatrix jJ2000ToTEME = MatrixUtils.createRealMatrix(jacJ2000ToTEME);
        final RealMatrix pTEME = jJ2000ToTEME.multiply(covWgsf1.multiplyTransposed(jJ2000ToTEME));

        final StateCovariance cov = new StateCovariance(pTEME, wgsf1.getDate(), FramesFactory.getTEME(), OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = tleProp.setupMatricesComputation(stmAdditionalName, null, null);
        final StateCovarianceMatrixProvider provider = 
                new StateCovarianceMatrixProvider("cov", stmAdditionalName, harvester, cov);
        tleProp.addAdditionalStateProvider(provider);

        // Propagate
        AbsoluteDate targetEpoch = new AbsoluteDate("2024-08-02T03:31:15.00000000000125Z", TimeScalesFactory.getUTC());
        final SpacecraftState finalState = tleProp.propagate(targetEpoch);
        final RealMatrix covProp = provider.getStateCovariance(finalState).getMatrix();
        PVCoordinates pvEme2000 = finalState.getPVCoordinates(j2000);
        //System.out.println(pvEme2000);

        // Convert covariance to eme2000
        final double[][] jacTEMEToJ2000 = new double[6][6];
        teme.getTransformTo(j2000, targetEpoch).getJacobian(CartesianDerivativesFilter.USE_PV, jacTEMEToJ2000);
        final RealMatrix jTEMEToJ2000 = MatrixUtils.createRealMatrix(jacTEMEToJ2000);
        final RealMatrix pJ2000 = jTEMEToJ2000.multiply(covProp.multiplyTransposed(jTEMEToJ2000));
        App.printCovariance(pJ2000);

        // Compute mahalanobis distance
        double[] meanMinusMean = new double[6];
        double[] meanStateProp = new double[]{pvEme2000.getPosition().getX(), pvEme2000.getPosition().getY(), pvEme2000.getPosition().getZ(), 
                                              pvEme2000.getVelocity().getX(), pvEme2000.getVelocity().getY(), pvEme2000.getVelocity().getZ()};
        double[] meanStateIod = new double[]{10238011.757933937, -7523573.5003033215, 692176.1997324437,
                                             284.92389251, 623.0602120194, -1.9274531721};
        for (int i=0; i<meanMinusMean.length; i++) {
            meanMinusMean[i] = meanStateIod[i] - meanStateProp[i];
        }

        // Compute inverse of covQ
        RealMatrix invCovQ = MatrixUtils.inverse(pJ2000);

        // Means transposed multiplied by inverse covariance of Q
        double[] meanTMultiplyInvCovQ = new double[]{0.,0.,0.,0.,0.,0.};
        for (int numCol=0; numCol<invCovQ.getColumnDimension(); numCol++) {
            double[] colInvCovQ = invCovQ.getColumn(numCol);
            for (int i=0; i<meanMinusMean.length; i++) {
                meanTMultiplyInvCovQ[numCol] += meanMinusMean[i] * colInvCovQ[i];
            }
        }
        
        // Multiply with mean again
        double meanTMultiplyInvCovQMultiplyMean = 0.;
        for (int i=0; i<meanMinusMean.length; i++) {
            meanTMultiplyInvCovQMultiplyMean += meanTMultiplyInvCovQ[i] * meanMinusMean[i];
        }

        double mahalanobis = FastMath.sqrt(meanTMultiplyInvCovQMultiplyMean);
        System.out.println("Distance: " + mahalanobis);
    } 
    private RealMatrix setCovariance(List<double[]> raw) {

        double[][] rawAsArray = new double[6][6];
        for(int row=0; row<raw.size(); row++) {
            double[] entireRow = raw.get(row);
            for(int col=0; col<entireRow.length; col++) { 
                rawAsArray[row][col] = entireRow[col];
                rawAsArray[col][row] = entireRow[col];
            }
        } 
        RealMatrix cov = new BlockRealMatrix(rawAsArray);
        return cov; 
    }

    @Test
    public void testMctsSearchAndTrack() {
        // Epoch
        AbsoluteDate current = new AbsoluteDate(2024, 7, 30, 3, 24, 0., TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = current.shiftedBy(60.* 60.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        //double slewT = 9.;
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        //double slewVel = fov.getHeight()/slewT;
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");
        Transform horizonToEci = topohorizon.getTransformTo(j2000, current);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(current, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        // Retrieve object of interest that shall be tracked
        List<ObservedObject> ooi = setListOOI(current);

        // Set environment storing the state of the tasking outputs
        List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
        stripeBullseyeCompleted.add(0); // Stripe scan
        stripeBullseyeCompleted.add(0); // Bullseye scan

        PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);

        // Initialise root node
        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));
        double initUtility = 1.;
        int numVisits = 1;
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC);
        double[] initWeights = new double[]{0.5, 0.5};
        double[] initTimeResources = 
            new double[]{initWeights[0] * endCampaign.durationFrom(current), 
                         initWeights[1] * endCampaign.durationFrom(current)};
        Node root = new DecisionNode(initUtility, numVisits, initPointing, initWeights, 
                                     initTimeResources, current, enviro, 0);
        MultiObjectiveMcts mctsTracking = 
            new MultiObjectiveMcts(root, objectives, current, endCampaign, "TDRS Station", ooi, 
                                   new ArrayList<ObservedObject>(), sensor);
        Node lastLeaf = mctsTracking.select(root);
        Node parent = lastLeaf.getParent();

        // Extract observable objects
        List<TLE> observables = getGEOSat(current);

        // Perform tasks
        List<AngularDirection> actualMeas = new ArrayList<AngularDirection>();

        while (!parent.equals(mctsTracking.getInitial())) {
            if(parent.getClass().getSimpleName().equals("ChanceNode")) {

                //System.out.println("Next node");
                ChanceNode parentChance = (ChanceNode)parent;
                if (parentChance.getMacro().getClass().getSimpleName().equals("SearchObjective")) {

                    List<AngularDirection> tasks = ((SearchObjective)parentChance.getMacro()).getScheduleGeocentric();
                    for(AngularDirection task: tasks) {
                        AbsoluteDate epoch = task.getDate();
                        double[] raRange = new double[]{task.getAngle1() - fov.getWidth()/2, 
                                                        task.getAngle1() + fov.getWidth()/2};
                        double[] decRange = new double[]{task.getAngle2() - fov.getHeight()/2,
                                                        task.getAngle2() + fov.getHeight()/2};
            
                        for(TLE candidate : observables) {
                            TLEPropagator prop = TLEPropagator.selectExtrapolator(candidate);
                            Vector3D propPos = prop.propagate(epoch).getPVCoordinates(j2000).getPosition();
                            AngularDirection anglePos = 
                                new AngularDirection(j2000, 
                                                    new double[]{propPos.getAlpha(), propPos.getDelta()}, 
                                                    AngleType.RADEC);
                            boolean inDecField = checkInAngularRange(anglePos, raRange, decRange);
            
                            // Extract measurement if object is in FOV
                            if(inDecField) {
                                anglePos.setDate(epoch);
                                actualMeas.add(anglePos);
                                //System.out.println("Detect: " + candidate.getSatelliteNumber() + " at " + epoch);
                            }
                        }
                    }
                } else {
                    TrackingObjective objective = (TrackingObjective)parentChance.getMacro();
                    long lastUpdated = objective.getLastUpdated();
                    AbsoluteDate epoch = parentChance.getEpoch();
                    //System.out.println("Track: " + lastUpdated + " at " + epoch + " frame: " + parentChance.getMicro().getFrame().getName());
                }
            }
            parent = parent.getParent();
        }
    }

    public List<ObservedObject> setListOOI(AbsoluteDate current) {
        // Frame
        Frame j2000 = FramesFactory.getEME2000();

        // Create list of objects of interest
        TLE tleTdrs05 = new TLE("1 21639U 91054B   24190.31993666 -.00000307  00000-0  00000-0 0  9990", 
                                "2 21639  14.1597 358.9539 0002937 207.3422 133.3222  1.00277803120606");      
        TLE tleTdrs06 = new TLE("1 22314U 93003B   24190.32793498 -.00000302  00000-0  00000-0 0  9994",
                                "2 22314  14.1631   2.2562 0011972 156.4976 200.1996  1.00268889115292");
        TLE tleTdrs12 = new TLE("1 39504U 14004A   24190.25733250 -.00000273  00000-0  00000-0 0  9996", 
                                "2 39504   3.5544   3.9152 0003777 189.8619 144.5768  1.00276604 37168");

        // Compute state
        TLEPropagator propTdrs05 = TLEPropagator.selectExtrapolator(tleTdrs05);
        TLEPropagator propTdrs06 = TLEPropagator.selectExtrapolator(tleTdrs06);
        TLEPropagator propTdrs12 = TLEPropagator.selectExtrapolator(tleTdrs12);

        SpacecraftState spacecraftTdrs05 = propTdrs05.propagate(current);
        SpacecraftState spacecraftTdrs06 = propTdrs06.propagate(current);
        SpacecraftState spacecraftTdrs12 = propTdrs12.propagate(current);

        StateVector stateTdrs05 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs05, j2000);
        StateVector stateTdrs06 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs06, j2000);
        StateVector stateTdrs12 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs12, j2000);

        double[][] covTdrs05 = new double[][]{{0.009855904759351372, 4.078127311069879E-7, 9.875741320556275E-8, 4.328235954545346E-6, 2.9071094424474166E-8, 7.50553956976887E-9},
                                              {4.078127311069879E-7, 0.009857835750718295, 2.03608156883521E-7, 2.98649724447625E-8, 4.306014824614407E-6, 6.301496942467037E-9},
                                              {9.875741320556275E-8, 2.03608156883521E-7, 0.009857086380024797, 7.70583422336434E-9, 6.298597666057694E-9, 4.283086978472644E-6},
                                              {4.328235954545346E-6, 2.98649724447625E-8, 7.70583422336434E-9, 1.7207377721346993E-8, -8.212503178246334E-10, -2.109541457159083E-10},
                                              {2.9071094424474166E-8, 4.306014824614407E-6, 6.298597666057694E-9, -8.212503178246334E-10, 1.7482251900748092E-8, -1.9276823004892938E-10},
                                              {7.50553956976887E-9, 6.301496942467037E-9, 4.283086978472644E-6, -2.109541457159083E-10, -1.9276823004892938E-10, 1.818503320357093E-8}};
        RealMatrix covMatrixTdrs05 = (new Array2DRowRealMatrix(covTdrs05)).scalarMultiply(1e6);
        System.out.println("Trace Tdrs5 :" + covMatrixTdrs05.getTrace());
        StateCovariance covEciTdrs05 = new StateCovariance(covMatrixTdrs05, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        double[][] covTdrs06 = new double[][]{{0.009855827555408531, 1.0287018380692445E-7, 3.910267644855593E-8, 4.318775659506618E-6, 3.082546456689512E-8, 7.3708924118463976E-9},
                                              {1.0287018380692445E-7, 0.009857910682561545, 2.2022442149622472E-7, 3.161755497050027E-8, 4.31510301473182E-6, 8.135870183215624E-9},
                                              {3.910267644855593E-8, 2.2022442149622472E-7, 0.009857088533581772, 7.570742600210252E-9, 8.144511266654299E-9, 4.2834614349166915E-6},
                                              {4.318775659506618E-6, 3.161755497050027E-8, 7.570742600210252E-9, 1.746275349153637E-8, -8.194811351938942E-10, -1.983493667495804E-10},
                                              {3.082546456689512E-8, 4.31510301473182E-6, 8.144511266654299E-9, -8.194811351938942E-10, 1.7236789470442757E-8, -2.4269048963433104E-10},
                                              {7.3708924118463976E-9, 8.135870183215624E-9, 4.2834614349166915E-6, -1.983493667495804E-10, -2.4269048963433104E-10, 1.8175062188421707E-8}};
        RealMatrix covMatrixTdrs06 = (new Array2DRowRealMatrix(covTdrs06)).scalarMultiply(1e6);
        System.out.println("Trace Tdrs6 :" + covMatrixTdrs06.getTrace());

        StateCovariance covEciTdrs06 = new StateCovariance(covMatrixTdrs06, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        double[][] covTdrs12 = new double[][]{{0.009855823497499241, -8.671670864459313E-8, 2.5731929906032663E-9, 4.312988285731713E-6, 3.1286742156174934E-8, 1.7377594871342673E-9},
                                              {-8.671670864459313E-8, 0.009857965268260518, 5.843499034914995E-8, 3.210358716654692E-8, 4.322741590079748E-6, 2.3545012547055407E-9},
                                              {2.5731929906032663E-9, 5.843499034914995E-8, 0.009857038107963734, 1.7885410467659128E-9, 2.3598727028774496E-9, 4.281608195160761E-6},
                                              {4.312988285731713E-6, 3.210358716654692E-8, 1.7885410467659128E-9, 1.7612360625313363E-8, -8.025283878518984E-10, -4.580050494765601E-11},
                                              {3.1286742156174934E-8, 4.322741590079748E-6, 2.3598727028774496E-9, -8.025283878518984E-10, 1.7031938068152258E-8, -6.947660259826142E-11},
                                              {1.7377594871342673E-9, 2.3545012547055407E-9, 4.281608195160761E-6, -4.580050494765601E-11, -6.947660259826142E-11, 1.8230352837789837E-8}};
        RealMatrix covMatrixTdrs12 = (new Array2DRowRealMatrix(covTdrs12)).scalarMultiply(1e6);
        System.out.println("Trace Tdrs12 :" + covMatrixTdrs12.getTrace());

        StateCovariance covEciTdrs12 = new StateCovariance(covMatrixTdrs12, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        CartesianCovariance stateCovTdrs05 =
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs05.getOrbit(), covEciTdrs05, j2000); 
        CartesianCovariance stateCovTdrs06 =
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs06.getOrbit(), covEciTdrs06, j2000);
        CartesianCovariance stateCovTdrs12 = 
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs12.getOrbit(), covEciTdrs12, j2000);

        // Create list of objects of interest
        ObservedObject tdrs05 = new ObservedObject(tleTdrs05.getSatelliteNumber(), stateTdrs05, stateCovTdrs05, current, j2000);
        ObservedObject tdrs06 = new ObservedObject(tleTdrs06.getSatelliteNumber(), stateTdrs06, stateCovTdrs06, current, j2000);
        ObservedObject tdrs12 = new ObservedObject(tleTdrs12.getSatelliteNumber(), stateTdrs12, stateCovTdrs12, current, j2000);

        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(tdrs05);
        ooi.add(tdrs06);
        ooi.add(tdrs12);

        return ooi;
    }


    @Test
    public void checkUncertaintyGrowth() {
        // Epoch
        AbsoluteDate current = new AbsoluteDate(2024, 7, 30, 3, 24, 0., TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = current.shiftedBy(60. * 30.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Ground station
        GeodeticPoint posStation = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, posStation, "TDRS Station");

        double readout = 7.;
        double exposure = 8.;
        double settling = 30.;
        double cutOff = FastMath.toRadians(5.);
        double slewT = 0.2;
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = fov.getHeight()/slewT;
        Sensor sensor = new Sensor("TDRS Station", fov, topohorizon.getPoint(), exposure, readout, 
                                    slewVel, settling, cutOff);
        
        List<ObservedObject> ooi =  setListOOI(current);
        for(ObservedObject candidate : ooi) {
            Vector3D pos = candidate.getState().getPositionVector();
            Vector3D vel = candidate.getState().getVelocityVector();
            PVCoordinates pv = new PVCoordinates(pos, vel);
            Orbit initialOrbit = new CartesianOrbit(pv, candidate.getFrame(), 
                                        candidate.getEpoch(), Constants.WGS84_EARTH_MU);
            KeplerianPropagator kepPropo = new KeplerianPropagator(initialOrbit);

            // Generate real measurement
            RealMatrix R = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow(1./206265, 2), 
                                                            FastMath.pow(1./206265, 2)});
            double[] residuals = new double[2];
            ObservedObject[] predAndCorr = 
                    TrackingObjective.estimateStateWithOwnExtendedKalman(kepPropo, endCampaign, R, 
                                                                         candidate, residuals, 
                                                                         sensor);
        }

    }
    @Test
    public void testOnlyTrack() throws IOException {
        long start = System.currentTimeMillis();
        
        // Epoch
        AbsoluteDate current = new AbsoluteDate(2024, 7, 30, 3, 24, 0., TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = current.shiftedBy(60. * 4.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");

        //Node lastLeaf = mcts.selectNew(mcts.getInitial());
        List<List<Node>> solutions = new ArrayList<List<Node>>();
        String out = "";

        // create FileWriter object with file as parameter 
        FileWriter outputfile = new FileWriter("Tuples_11.csv"); 
    
        // create CSVWriter object filewriter object as parameter 
        CSVWriter writer = new CSVWriter(outputfile); 
        List<double[]> utilityStrategiesRatio = new ArrayList<double[]>();

        int mctsIterations = 10;
        double weight = 1./3.;

        for (int i=0; i<mctsIterations; i++) {
            System.out.println(" MCTS number " + i);
             // Set environment storing the state of the tasking outputs
            List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
            stripeBullseyeCompleted.add(0); // Stripe scan
            stripeBullseyeCompleted.add(0); // Bullseye scan

            List<ObservedObject> ooi =  setListOOI(current);

            final PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);

            double[] initWeights = new double[]{0., 1.};

            MultiObjectiveMcts mcts = setUpMcts(current, endCampaign, topohorizon, enviro, initWeights);

            List<Node> strategy = mcts.run(9000);
            String[] selected = new String[(strategy.size()-1)/2 + 1 + ooi.size()];
            int j =0;
            for(Node currentNode : strategy) {
                if (currentNode.getClass().getSimpleName().equals("ChanceNode")) {
                    String objective = ((ChanceNode) currentNode).getMacro().getClass().getSimpleName();
                    if (objective.equals("TrackingObjective")) {
                        long id = ((TrackingObjective)((ChanceNode) currentNode).getMacro())
                                                                                    .getLastUpdated();

                        if (id == 21639) {
                            out = out + "A ";

                        } else if(id == 22314) {
                            out = out + "B ";
                        } else {
                            out = out + "C ";
                        }
                        selected[j] = Long.toString(id);
                        System.out.print(selected[j] + " ");
                        j++;
                        
                    } else {
                        out = out + "S ";
                        selected[j] = "S";
                        System.out.print(selected[j] + " ");
                        j++;
                    }
                } else {
                    //j=0;
                    continue;
                }
            }
            // Compute IG of final strategy
            double[] iG = mcts.computeTrackReward((DecisionNode)strategy.get(strategy.size()-1));
            for(int k=0; k<iG.length; k++) {
                selected[j + k] = Double.toString(iG[k]);
                System.out.print(selected[j + k] + " - ");
            }

            // Calculate ratio 
            double[] ratioUtility = new double[iG.length];
            double totalIG = 0.;
            for(int k=0; k<iG.length; k++) {
                totalIG += iG[k];
            }
            for(int k=0; k<iG.length; k++) {
                ratioUtility[k] = FastMath.abs((iG[k]/totalIG) - weight);
            }
            utilityStrategiesRatio.add(ratioUtility);
            System.out.println();

            writer.writeNext(selected);
            j=0;
            out = out + "\n";
            solutions.add(strategy);
        }
        System.out.println(out);
        List<Integer> index = new ArrayList<Integer>();
        double max = Double.NEGATIVE_INFINITY;
        // Filter strategies
        for(int i=0; i<mctsIterations; i++) {
            double[] removedUtility = utilityStrategiesRatio.remove(0);
            OptimisingVector opt = new OptimisingVector(utilityStrategiesRatio, removedUtility.length - 1);
            List<double[]> domVecs = 
                opt.getDominatingVecs(removedUtility, 
                                      new boolean[]{false, false, false}, 
                                      0);
            int utility = - domVecs.size();
            if(utility>max) {
                max = utility;
                index.clear();
                index.add(i);
            } else if (utility==max) {
                index.add(i);
            }
            utilityStrategiesRatio.add(removedUtility);
        }
        long finish = System.currentTimeMillis();
        long elapsed = finish - start;

        System.out.println("Following strategies considered as optimal:");
        for(Integer i : index) {
            System.out.print(i + " ");
        }
        System.out.println("Time " + elapsed + " ms");

        // closing writer connection 
        writer.close(); 


    }
    @Test
    public void testOnlySearch() {

        // Epoch
        AbsoluteDate current = new AbsoluteDate(2024, 7, 30, 3, 24, 0., TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = current.shiftedBy(60. * 20.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");

        // Set environment storing the state of the tasking outputs
        List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
        stripeBullseyeCompleted.add(0); // Stripe scan
        stripeBullseyeCompleted.add(0); // Bullseye scan

        List<ObservedObject> ooi = setListOOI(current);

        PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);

        double[] initWeights = new double[]{1., 0.};

        MultiObjectiveMcts mcts = setUpMcts(current, endCampaign, topohorizon, enviro, initWeights);

        Node lastLeaf = mcts.select(mcts.getInitial());

        // Check that each node only has one child
        Assert.assertEquals(1, mcts.getInitial().getChildren().size());

        Node currentChild = mcts.getInitial().getChildren().get(0);
        while(currentChild.getId() != lastLeaf.getId()) {
            Assert.assertEquals(1, currentChild.getChildren().size());
            currentChild = currentChild.getChildren().get(0);
        }

        // Check time stamp for first search 
        ChanceNode search = (ChanceNode)mcts.getInitial().getChildren().get(0);
        checkSearchNode(search, current, topohorizon, 195.750408);
                
        
        // Check time stamp for second search task
        current = search.getChildren().get(0).getEpoch();
        ChanceNode search2 = (ChanceNode)lastLeaf.getParent();
        checkSearchNode(search2, current, topohorizon, 31.608597);

    }
                
    private MultiObjectiveMcts setUpMcts(AbsoluteDate current, AbsoluteDate endCampaign, 
                                         TopocentricFrame topohorizon, 
                                         PropoagatedEnvironment enviro,
                                         double[] initWeights) {
        
        double readout = 7.;
        double exposure = 8.;
        double settling = 30.;
        double cutOff = FastMath.toRadians(5.);
        double slewT = 0.2;
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = fov.getHeight()/slewT;
        Sensor sensor = new Sensor("TDRS Station", fov, topohorizon.getPoint(), exposure, readout, 
                                    slewVel, settling, cutOff);

        Transform horizonToEci = topohorizon.getTransformTo(j2000, current);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(current, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        // Initialise root node
        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));
        double initUtility = 1.;
        int numVisits = 1;
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC);
        double[] initTimeResources = new double[]{endCampaign.durationFrom(current), 0.};

        
        Node root = new DecisionNode(initUtility, numVisits, initPointing, initWeights, 
                                     initTimeResources, current, enviro, 0);
        MultiObjectiveMcts mcts = 
            new MultiObjectiveMcts(root, objectives, current, endCampaign, "TDRS Station", null, 
                                   new ArrayList<ObservedObject>(), sensor);
        return mcts;
        }
        
    private void checkSearchNode(ChanceNode search, AbsoluteDate current, 
                            TopocentricFrame topohorizon, double reloc) {
        double geoDistance = Constants.WGS84_EARTH_EQUATORIAL_RADIUS + 35786 * 1e3;  // in m

        List<AngularDirection> actualTopo = 
            ((SearchObjective)search.getMacro()).getScheduleTopocentric();
        List<AngularDirection> actualGeo = 
            ((SearchObjective)search.getMacro()).getScheduleGeocentric();
        // Shift by reloc, settling, prep and half of expose time
        AbsoluteDate instance = current.shiftedBy(reloc + 30. + 6. + 4.);
        //Check in time stamps and topocentric angles regarding first dec field
        for(int i=0; i<actualTopo.size(); i++) {

            // Compute topocentric inertial frame
            Transform horizonToEci = topohorizon.getTransformTo(j2000, instance);  // date has to be the measurement epoch
            Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
            Transform eciToTopo = new Transform(instance, coordinatesStationEci.negate());
            Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);
            AngularDirection trans = 
                actualGeo.get(i).transformReference(topocentric, instance, AngleType.RADEC, 
                                                    geoDistance);
            Assert.assertEquals(actualTopo.get(i).getAngle1(), trans.getAngle1(), 1e-9);
            Assert.assertEquals(actualTopo.get(i).getAngle2(), trans.getAngle2(), 1e-9);
            Assert.assertEquals(0., instance.durationFrom(actualTopo.get(i).getDate()), 1e-6);
            // shift by expose and read out time
            instance = instance.shiftedBy(8. + 7.);

            int j = i+1;
            if(j%5==0) {
                // shift by repos and prep time and substract readout
                instance = instance.shiftedBy(-7. + 39. + 6);
            }
        }
    }
        
    private static boolean checkInAngularRange(AngularDirection obj, double[] raRange, double[] decRange) {
        double ra = obj.getAngle1() ;
        if(ra< 0.) {
            ra += 2*FastMath.PI;
        }

        if(raRange[0] < ra && ra < raRange[1] 
                && decRange[0] < obj.getAngle2() && obj.getAngle2() < decRange[1]) {
                    return true;
            }
        return false;
    }
    public List<TLE> getGEOSat(AbsoluteDate current){

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        String workingDir = System.getProperty("user.dir");
        String nameFile = "\\geo_02082024.txt";
        List<TLE> filtered = new ArrayList<TLE>();
        try (BufferedReader br = new BufferedReader(new FileReader(workingDir + nameFile))) {
            String line;
            String line1 = "";
            while ((line = br.readLine()) != null) {
                if(line.startsWith("1")) {
                    line1 = line;
                } else if(line.startsWith("2")) {
                    TLE candidate = new TLE(line1, line);
                    TLEPropagator prop = TLEPropagator.selectExtrapolator(candidate);
                    SpacecraftState state = prop.propagate(current);
                    PVCoordinates pvEcef =  state.getPVCoordinates(ecef);
                    double lon = FastMath.toDegrees(pvEcef.getPosition().getAlpha());
                    double lat = FastMath.toDegrees(pvEcef.getPosition().getDelta());

                    // Longitude and latitude filter
                    //if((-10. <= lat && lat <= 15.) && (-50. <= lon && lon <= -25.)) {
                        filtered.add(candidate);
                    //}
                }
            }
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        //System.out.println("Number of observable satellites: " + filtered.size());
        return filtered;
    }

    @Test
    public void testSelectTdrs() {

        // Epoch
        AbsoluteDate current = new AbsoluteDate(2024, 7, 30, 3, 24, 0., TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = current.shiftedBy(60.*20.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        double readout = 7.;
        double exposure = 8.;
        double settling = 30.;
        double cutOff = FastMath.toRadians(5.);
        double slewT = 9.;
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = fov.getHeight()/slewT;
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);

        // Create list of objects of interest
        TLE tleTdrs05 = new TLE("1 21639U 91054B   24190.31993666 -.00000307  00000-0  00000-0 0  9990", 
                                "2 21639  14.1597 358.9539 0002937 207.3422 133.3222  1.00277803120606");      
        TLE tleTdrs06 = new TLE("1 22314U 93003B   24190.32793498 -.00000302  00000-0  00000-0 0  9994",
                                "2 22314  14.1631   2.2562 0011972 156.4976 200.1996  1.00268889115292");
        TLE tleTdrs12 = new TLE("1 39504U 14004A   24190.25733250 -.00000273  00000-0  00000-0 0  9996", 
                                "2 39504   3.5544   3.9152 0003777 189.8619 144.5768  1.00276604 37168");
        
        // Compute state
        TLEPropagator propTdrs05 = TLEPropagator.selectExtrapolator(tleTdrs05);
        TLEPropagator propTdrs06 = TLEPropagator.selectExtrapolator(tleTdrs06);
        TLEPropagator propTdrs12 = TLEPropagator.selectExtrapolator(tleTdrs12);

        SpacecraftState spacecraftTdrs05 = propTdrs05.propagate(current);
        SpacecraftState spacecraftTdrs06 = propTdrs06.propagate(current);
        SpacecraftState spacecraftTdrs12 = propTdrs12.propagate(current);

        StateVector stateTdrs05 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs05, j2000);
        StateVector stateTdrs06 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs06, j2000);
        StateVector stateTdrs12 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs12, j2000);
        //StateVector stateTdrs13 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs13, j2000);

        // Set covariances
/*         double[] covDiag = new double[]{100., 100., 100., 1., 1., 1.};
        RealMatrix covMatrix = MatrixUtils.createRealDiagonalMatrix(covDiag);
        StateCovariance covEci = new StateCovariance(covMatrix, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN); */
/*         double[] covTdrs05 = new double[]{100., 100., 100., 1., 1., 1.};
        RealMatrix covMatrixTdrs05 = new DiagonalMatrix(covTdrs05); */

        double[][] covTdrs05 = new double[][]{{0.009855904759351372, 4.078127311069879E-7, 9.875741320556275E-8, 4.328235954545346E-6, 2.9071094424474166E-8, 7.50553956976887E-9},
                                              {4.078127311069879E-7, 0.009857835750718295, 2.03608156883521E-7, 2.98649724447625E-8, 4.306014824614407E-6, 6.301496942467037E-9},
                                              {9.875741320556275E-8, 2.03608156883521E-7, 0.009857086380024797, 7.70583422336434E-9, 6.298597666057694E-9, 4.283086978472644E-6},
                                              {4.328235954545346E-6, 2.98649724447625E-8, 7.70583422336434E-9, 1.7207377721346993E-8, -8.212503178246334E-10, -2.109541457159083E-10},
                                              {2.9071094424474166E-8, 4.306014824614407E-6, 6.298597666057694E-9, -8.212503178246334E-10, 1.7482251900748092E-8, -1.9276823004892938E-10},
                                              {7.50553956976887E-9, 6.301496942467037E-9, 4.283086978472644E-6, -2.109541457159083E-10, -1.9276823004892938E-10, 1.818503320357093E-8}};
        RealMatrix covMatrixTdrs05 = (new Array2DRowRealMatrix(covTdrs05)).scalarMultiply(1e3);
        StateCovariance covEciTdrs05 = new StateCovariance(covMatrixTdrs05, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        double[][] covTdrs06 = new double[][]{{0.009855827555408531, 1.0287018380692445E-7, 3.910267644855593E-8, 4.318775659506618E-6, 3.082546456689512E-8, 7.3708924118463976E-9},
                                              {1.0287018380692445E-7, 0.009857910682561545, 2.2022442149622472E-7, 3.161755497050027E-8, 4.31510301473182E-6, 8.135870183215624E-9},
                                              {3.910267644855593E-8, 2.2022442149622472E-7, 0.009857088533581772, 7.570742600210252E-9, 8.144511266654299E-9, 4.2834614349166915E-6},
                                              {4.318775659506618E-6, 3.161755497050027E-8, 7.570742600210252E-9, 1.746275349153637E-8, -8.194811351938942E-10, -1.983493667495804E-10},
                                              {3.082546456689512E-8, 4.31510301473182E-6, 8.144511266654299E-9, -8.194811351938942E-10, 1.7236789470442757E-8, -2.4269048963433104E-10},
                                              {7.3708924118463976E-9, 8.135870183215624E-9, 4.2834614349166915E-6, -1.983493667495804E-10, -2.4269048963433104E-10, 1.8175062188421707E-8}};
        /* double[] covTdrs06 = new double[]{110., 110., 110., 1.0, 1.0, 1.0};
        RealMatrix covMatrixTdrs06 = new DiagonalMatrix(covTdrs06); */
        RealMatrix covMatrixTdrs06 = (new Array2DRowRealMatrix(covTdrs06)).scalarMultiply(1e3);
        StateCovariance covEciTdrs06 = new StateCovariance(covMatrixTdrs06, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        double[][] covTdrs12 = new double[][]{{0.009855823497499241, -8.671670864459313E-8, 2.5731929906032663E-9, 4.312988285731713E-6, 3.1286742156174934E-8, 1.7377594871342673E-9},
                                              {-8.671670864459313E-8, 0.009857965268260518, 5.843499034914995E-8, 3.210358716654692E-8, 4.322741590079748E-6, 2.3545012547055407E-9},
                                              {2.5731929906032663E-9, 5.843499034914995E-8, 0.009857038107963734, 1.7885410467659128E-9, 2.3598727028774496E-9, 4.281608195160761E-6},
                                              {4.312988285731713E-6, 3.210358716654692E-8, 1.7885410467659128E-9, 1.7612360625313363E-8, -8.025283878518984E-10, -4.580050494765601E-11},
                                              {3.1286742156174934E-8, 4.322741590079748E-6, 2.3598727028774496E-9, -8.025283878518984E-10, 1.7031938068152258E-8, -6.947660259826142E-11},
                                              {1.7377594871342673E-9, 2.3545012547055407E-9, 4.281608195160761E-6, -4.580050494765601E-11, -6.947660259826142E-11, 1.8230352837789837E-8}};
/*         double[] covTdrs12 = new double[]{90., 90., 90., 1.0, 1.0, 1.0};
        RealMatrix covMatrixTdrs12 = new DiagonalMatrix(covTdrs12); */
        RealMatrix covMatrixTdrs12 = (new Array2DRowRealMatrix(covTdrs12)).scalarMultiply(1e3);
        StateCovariance covEciTdrs12 = new StateCovariance(covMatrixTdrs12, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        
        CartesianCovariance stateCovTdrs05 =
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs05.getOrbit(), covEciTdrs05, j2000); 
        CartesianCovariance stateCovTdrs06 =
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs06.getOrbit(), covEciTdrs06, j2000);
        CartesianCovariance stateCovTdrs12 = 
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs12.getOrbit(), covEciTdrs12, j2000);

        // Create list of objects of interest
        ObservedObject tdrs05 = new ObservedObject(tleTdrs05.getSatelliteNumber(), stateTdrs05, stateCovTdrs05, current, j2000);
        ObservedObject tdrs06 = new ObservedObject(tleTdrs06.getSatelliteNumber(), stateTdrs06, stateCovTdrs06, current, j2000);
        ObservedObject tdrs12 = new ObservedObject(tleTdrs12.getSatelliteNumber(), stateTdrs12, stateCovTdrs12, current, j2000);
        //ObservedObject tdrs13 = new ObservedObject(tleTdrs13.getSatelliteNumber(), stateTdrs13, stateCovTdrs13, current, j2000);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(tdrs05);
        ooi.add(tdrs06);
        ooi.add(tdrs12);

        // Set environment storing the state of the tasking outputs
        List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
        stripeBullseyeCompleted.add(0); // Stripe scan
        stripeBullseyeCompleted.add(0); // Bullseye scan

        PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "Station to observe TDRS satellite");
        Transform horizonToEci = topohorizon.getTransformTo(j2000, current);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(current, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));

        // Initialise root node
        double initUtility = 1.;
        int numVisits = 1;
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC);
        double[] initWeights = new double[]{0., 1.};
        double[] initTimeResources = new double[]{0., endCampaign.durationFrom(current)};
        Node root = new DecisionNode(initUtility, numVisits, initPointing, initWeights, 
                                     initTimeResources, current, enviro, 0);
        MultiObjectiveMcts mctsTracking = 
            new MultiObjectiveMcts(root, objectives, current, endCampaign, "TDRS Station", ooi, 
                                   new ArrayList<ObservedObject>(), sensor);
        Node lastLeaf = mctsTracking.select(root);
        //System.out.println(rootUpdated.getUtility());
    }

    @Test
    public void testComputeSearchReward() {
        // Build up test decision tree
        DecisionNode root = new DecisionNode(1, 1, null, null, null, new AbsoluteDate(), null, 0);

        // Stripe scan 70% and Bullseye scan 30%
        root.setWeightsSearchingTask(new double[]{0.7, 0.3});

        // Set up decision tree
        List<Integer> task1 = new ArrayList<>();
        task1.add(1);
        task1.add(0);
        SearchObjective search1 = new SearchObjective(task1, null, null, 0, null);
        ChanceNode c1 = new ChanceNode(null, 1, 1, search1, null, root, root.incrementIdCounter());
        DecisionNode d1 = new DecisionNode(0, 1, null, null, null, null, 
                                           new PropoagatedEnvironment(null, task1),
                                           root.incrementIdCounter());
        c1.setChild(d1);

        List<Integer> task2 = new ArrayList<>();
        task2.add(2);
        task2.add(0);
        SearchObjective search2 = new SearchObjective(task2, null, null, 0, null);
        ChanceNode c2 = new ChanceNode(null, 1, 1, search2, null, d1, root.incrementIdCounter());
        DecisionNode d2 = new DecisionNode(0, 1, null, null, null, null, 
                                           new PropoagatedEnvironment(null, task2),
                                           root.incrementIdCounter());
        c2.setChild(d2);

        List<Integer> task31 = new ArrayList<>();
        task31.add(3);
        task31.add(0);
        SearchObjective search31 = new SearchObjective(task31, null, null, 0, null);
        ChanceNode c31 = new ChanceNode(null, 1, 1, search31, null, d2, root.incrementIdCounter());
        DecisionNode d31 = new DecisionNode(1, 1, null, null, null, null, 
                                            new PropoagatedEnvironment(null, task31),
                                            root.incrementIdCounter());
        c31.setChild(d31);

        // Set up discrepancy vectors of leaf nodes
        long extractedId = d2.getId();
        root.addSearchDiscrepancyVec(Long.valueOf(extractedId), new double[]{0., 0.});
        root.addSearchDiscrepancyVec(Long.valueOf(10), new double[]{0.03, 0.03});
        root.addSearchDiscrepancyVec(Long.valueOf(11), new double[]{0.03, 0.03});
        root.addSearchDiscrepancyVec(Long.valueOf(12), new double[]{0.37, 0.37});
        root.addSearchDiscrepancyVec(Long.valueOf(13), new double[]{0.03, 0.03});
        root.addSearchDiscrepancyVec(Long.valueOf(14), new double[]{0.37, 0.37});
        root.addSearchDiscrepancyVec(Long.valueOf(15), new double[]{0.37, 0.37});
        root.addSearchDiscrepancyVec(Long.valueOf(16), new double[]{0.7, 0.7});

        AbsoluteDate current = (new AbsoluteDate(2024, 8, 2, 3, 24, 0., 
                                                 TimeScalesFactory.getUTC())).shiftedBy(4000.);
        AbsoluteDate endCampaign = current.shiftedBy(10.* 60.);
        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                     0.);              // in [m]

                                     double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);

        List<ObservedObject> ooiAll = setListOOI(current);
        MultiObjectiveMcts mcts = new MultiObjectiveMcts(root, objectives, current, endCampaign, 
                                                         "TDRS Station", ooiAll, 
                                                         new ArrayList<ObservedObject>(), sensor);
        double actualReward = mcts.computeSearchReward(d31, d31);

        Assert.assertEquals(-3., actualReward, 1e-16);
    }

    @Test
    public void testComputeTrackReward(){

        // Dates
        AbsoluteDate initDate = AbsoluteDate.J2000_EPOCH.shiftedBy(584.);
        AbsoluteDate target = new AbsoluteDate("2000-01-01T15:50:04.36035985407872Z", TimeScalesFactory.getUTC());

        // Set up targets initial state
        Vector3D posA = new Vector3D(7.0e6, 1.0e6, 4.0e6);
        Vector3D velA = new Vector3D(-500.0, 8000.0, 1000.0);
        Orbit orbitA = new CartesianOrbit(new PVCoordinates(posA, velA), j2000, initDate, 
                                          Constants.WGS84_EARTH_MU);
        SpacecraftState stateA = new SpacecraftState(orbitA);
        StateVector stateVecA = ObservedObject.spacecraftStateToStateVector(stateA, j2000);
        RealMatrix covInitMatrixA = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                              0.1, 0.1, 0.1});
        StateCovariance covA = new StateCovariance(covInitMatrixA, initDate, j2000, 
                                                   OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovA = ObservedObject.stateCovToCartesianCov(orbitA, covA, j2000);
        ObservedObject objA = new ObservedObject(11111, stateVecA, cartCovA, initDate, j2000);

        Vector3D posB = new Vector3D(7.1e6, 1.0e6, 3.9e6);
        Vector3D velB = new Vector3D(-500.1, 8000.0, 999.9);
        Orbit orbitB = new CartesianOrbit(new PVCoordinates(posB, velB), j2000, initDate, 
                                          Constants.WGS84_EARTH_MU);
        SpacecraftState stateB = new SpacecraftState(orbitB);
        StateVector stateVecB = ObservedObject.spacecraftStateToStateVector(stateB, j2000);
        RealMatrix covInitMatrixB = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{99*1e3, 100*1e3, 101*1e3, 
                                                              0.2, 0.2, 0.2});
        StateCovariance covB = new StateCovariance(covInitMatrixB, initDate, j2000, 
                                                   OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovB = ObservedObject.stateCovToCartesianCov(orbitB, covB, j2000);
        ObservedObject objB = new ObservedObject(22222, stateVecB, cartCovB, initDate, j2000);

        // Set up targets updated state
        Vector3D posUpdatedA = posA;
        Vector3D velUpdatedA = velA;
        Orbit orbitUpdatedA = new CartesianOrbit(new PVCoordinates(posUpdatedA, velUpdatedA),  
                                                 j2000, target, Constants.WGS84_EARTH_MU);
        SpacecraftState stateUpdatedA = new SpacecraftState(orbitUpdatedA);
        StateVector stateVecUpdatedA = 
            ObservedObject.spacecraftStateToStateVector(stateUpdatedA, j2000);

        double[][] covArrayA = new double[][]{{5.402139881e+04,  7.715437798e+03,  3.158651844e+04, -4.062297642e+00, -4.184765975e+01, -8.887067576e+00},
                                            {7.715437798e+03,  1.156989368e+03,  4.515760025e+03, -6.156051795e-01, -5.983722439e+00, -1.289856904e+00},
                                            {3.158651844e+04,  4.515760025e+03,  1.856532417e+04, -2.385340451e+00, -2.450072498e+01, -5.205936632e+00},
                                            {-4.062297642e+00, -6.156051795e-01, -2.385340451e+00,  1.451090187e-01,  1.174814384e-02,  2.653099702e-02},
                                            {-4.184765975e+01, -5.983722439e+00, -2.450072498e+01,  1.174814384e-02,  3.859792809e-02, -3.367440602e-03},
                                            {-8.887067576e+00, -1.289856904e+00, -5.205936632e+00,  2.653099702e-02, -3.367440602e-03,  1.141828017e-01}};
        RealMatrix covMatrixA = new Array2DRowRealMatrix(covArrayA);
        StateCovariance covUpdatedA = new StateCovariance(covMatrixA, target, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        
        CartesianCovariance cartCovUpdatedA = 
            ObservedObject.stateCovToCartesianCov(orbitUpdatedA, covUpdatedA, j2000);
        ObservedObject objUpdatedA = 
            new ObservedObject(11111, stateVecUpdatedA, cartCovUpdatedA, target, j2000);

        Vector3D posUpdatedB = new Vector3D(7.026977162e+06, -1.264918180e+06, 3.503803802e+06);
        Vector3D velUpdatedB = new Vector3D(1.034823223e+03, 7.968025511e+03, 1.803833605e+03);
        Orbit orbitUpdatedB = new CartesianOrbit(new PVCoordinates(posUpdatedB, velUpdatedB), 
                                                 j2000, target, Constants.WGS84_EARTH_MU);
        SpacecraftState stateUpdatedB = new SpacecraftState(orbitUpdatedB);
        StateVector stateVecUpdatedB = 
            ObservedObject.spacecraftStateToStateVector(stateUpdatedB, j2000);

        double[][] covArrayB = new double[][]{{8.243199250e+04, -1.482804669e+04,  4.219116446e+04, -6.399749455e+01, -5.088743142e+01, -4.176143991e+01},
                                            {-1.482804669e+04,  2.715566494e+03, -7.593739807e+03,  1.148752913e+01,  9.165024829e+00,  7.504224162e+00},
                                            {4.219116446e+04, -7.593739807e+03,  2.166857172e+04, -3.277663953e+01, -2.605994761e+01, -2.140814806e+01},
                                            {-6.399749455e+01,  1.148752913e+01, -3.277663953e+01,  2.534003488e-01,  1.806549691e-02,  3.968955271e-02},
                                            {-5.088743142e+01,  9.165024829e+00, -2.605994761e+01,  1.806549691e-02,  4.591038606e-02, -1.234796175e-02},
                                            {-4.176143991e+01,  7.504224162e+00, -2.140814806e+01,  3.968955271e-02, -1.234796175e-02,  2.030162605e-01}};
        RealMatrix covMatrixB = new Array2DRowRealMatrix(covArrayB);
        StateCovariance covUpdatedB = 
            new StateCovariance(covMatrixB, target, j2000, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN);
        CartesianCovariance cartCovUpdatedB = 
            ObservedObject.stateCovToCartesianCov(orbitUpdatedB, covUpdatedB, j2000);
        ObservedObject objUpdatedB = 
            new ObservedObject(22222, stateVecUpdatedB, cartCovUpdatedB, target, j2000);

        // Set up nodes
        List<ObservedObject> targetsInit = new ArrayList<ObservedObject>();
        targetsInit.add(objA);
        targetsInit.add(objB);
        PropoagatedEnvironment env = 
            new PropoagatedEnvironment(targetsInit, new ArrayList<Integer>());
        DecisionNode root = 
            new DecisionNode(1, 1, null, null, null, new AbsoluteDate(), env, 0);

        List<ObservedObject> targetsUpdated = new ArrayList<ObservedObject>();
        targetsUpdated.add(objUpdatedA);
        targetsUpdated.add(objUpdatedB);
        PropoagatedEnvironment envUpdated = 
            new PropoagatedEnvironment(targetsUpdated, new ArrayList<Integer>());
        DecisionNode leaf = 
            new DecisionNode(0, 0, null, null, null, target, envUpdated, 0);

        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                                Constants.WGS84_EARTH_FLATTENING,
                                                ecef);

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                     0.);              // in [m]

        double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);

        MultiObjectiveMcts mcts = new MultiObjectiveMcts(root, objectives, initDate, target, 
                                                         "TDRS Station", targetsInit, 
                                                         new ArrayList<ObservedObject>(), sensor);
        /* double actualReward = mcts.computeTrackReward(leaf);
        Assert.assertEquals(2.7682235805565E7, actualReward, 1e-16); */
   }

   @Test
   public void testBackpropagate() {
        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                            FastMath.toRadians(-37.),   // Longitude
                                    0.);              // in [m]

        double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);
        
        // Dates
        AbsoluteDate initDate = AbsoluteDate.J2000_EPOCH.shiftedBy(584.);

        // Set up targets initial state
        Vector3D posA = new Vector3D(7.0e6, 1.0e6, 4.0e6);
        Vector3D velA = new Vector3D(-500.0, 8000.0, 1000.0);
        Orbit orbit1 = new CartesianOrbit(new PVCoordinates(posA, velA), j2000, initDate, 
                                        Constants.WGS84_EARTH_MU);
        SpacecraftState stateA = new SpacecraftState(orbit1);
        StateVector stateVecA = ObservedObject.spacecraftStateToStateVector(stateA, j2000);
        RealMatrix covInitMatrixA = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                            0.1, 0.1, 0.1});
        StateCovariance covA = new StateCovariance(covInitMatrixA, initDate, j2000, 
                                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovA = ObservedObject.stateCovToCartesianCov(orbit1, covA, j2000);
        ObservedObject obj1 = new ObservedObject(11111, stateVecA, cartCovA, initDate, j2000);

        Vector3D posB = new Vector3D(7.1e6, 1.0e6, 3.9e6);
        Vector3D velB = new Vector3D(-500.1, 8000.0, 999.9);
        Orbit orbit2 = new CartesianOrbit(new PVCoordinates(posB, velB), j2000, initDate, 
                                        Constants.WGS84_EARTH_MU);
        SpacecraftState stateB = new SpacecraftState(orbit2);
        StateVector stateVecB = ObservedObject.spacecraftStateToStateVector(stateB, j2000);
        RealMatrix covInitMatrixB = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{99*1e3, 100*1e3, 101*1e3, 
                                                            0.2, 0.2, 0.2});
        StateCovariance covB = new StateCovariance(covInitMatrixB, initDate, j2000, 
                                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovB = ObservedObject.stateCovToCartesianCov(orbit2, covB, j2000);
        ObservedObject obj2 = new ObservedObject(22222, stateVecB, cartCovB, initDate, j2000);

        // Set root node
        List<ObservedObject> targetsInit = new ArrayList<ObservedObject>();
        targetsInit.add(obj1);
        targetsInit.add(obj2);
        PropoagatedEnvironment env = 
            new PropoagatedEnvironment(targetsInit, new ArrayList<Integer>());
        DecisionNode root = 
            new DecisionNode(1, 1, null, null, null, initDate, env, 0);
        // 70% on search capacity shall be spent on stripe, 30% on bullseye scan
        root.setWeightsSearchingTask(new double[]{0.7, 0.3});   
        
        // Set nodes a
        AbsoluteDate epochA = initDate.shiftedBy(10. * 60.);
        List<Integer> taskA = new ArrayList<>();
        taskA.add(1);
        taskA.add(0);
        SearchObjective search1 = new SearchObjective(taskA, null, null, 0, null);
        ChanceNode chanceA = 
            new ChanceNode(null, 1, 1, search1, null, root, root.incrementIdCounter());
        DecisionNode decisionA = new DecisionNode(0, 1, null, null, null, epochA, 
                                                new PropoagatedEnvironment(targetsInit, taskA),
                                                root.incrementIdCounter());
        chanceA.setChild(decisionA);

        AbsoluteDate epochAPrime = epochA.shiftedBy(2. * 60.);
        KeplerianPropagator kepPropo = new KeplerianPropagator(orbit1);
        RealMatrix R = 
                MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow(1./206265, 2), 
                                                                    FastMath.pow(1./206265, 2)});
        ObservedObject[] predAndCorr = 
                TrackingObjective.estimateStateWithOwnExtendedKalman(kepPropo, epochAPrime, R, obj1, new double[2], sensor);
        List<ObservedObject> targetsPropA = new ArrayList<ObservedObject>();
        targetsPropA.add(predAndCorr[1]);
        targetsPropA.add(obj2);
        TrackingObjective track1 = new TrackingObjective(targetsInit, sensor, epochAPrime);
        ChanceNode chanceAPrime = 
            new ChanceNode(null, 1, 1, track1, null, decisionA, root.incrementIdCounter());
        DecisionNode decisionAPrime = new DecisionNode(0, 1, null, null, null, epochAPrime, 
                                                       new PropoagatedEnvironment(targetsPropA, taskA), 
                                                       root.incrementIdCounter());
        DecisionNode.setParent(decisionAPrime, chanceAPrime);

        // Set MCTS
        List<String> objectives = new ArrayList<String>(Arrays.asList("SEARCH", "TRACK"));
        MultiObjectiveMcts mcts = 
            new MultiObjectiveMcts(root, objectives, initDate, epochAPrime, "TDRS Station", 
                                   targetsInit, new ArrayList<ObservedObject>(), sensor);
        mcts.backpropagate(decisionA, decisionAPrime);

        // Compare
        Assert.assertEquals(1., root.getUtility(), 1e-16);
        Assert.assertEquals(2, root.getNumVisits());
        Assert.assertEquals(1., chanceA.getUtility(), 1e-16);
        Assert.assertEquals(2, chanceA.getNumVisits());
        Assert.assertEquals(0., decisionA.getUtility(), 1e-16);
        Assert.assertEquals(2, decisionA.getNumVisits());

        // Set nodes b
        AbsoluteDate epochB = initDate.shiftedBy(2. * 60.);
        KeplerianPropagator kepPropo1 = new KeplerianPropagator(orbit1);
        ObservedObject[] predAndCorrB = 
            TrackingObjective.estimateStateWithOwnExtendedKalman(kepPropo1, epochB, R, obj1, new double[2], sensor);
        List<Integer> taskB = new ArrayList<>();
        taskB.add(0);
        taskB.add(0);
        List<ObservedObject> targetsPropB = new ArrayList<ObservedObject>();
        targetsPropB.add(predAndCorrB[1]);
        targetsPropB.add(obj2);
        ChanceNode chanceB =
            new ChanceNode(null, 1, 1, track1, null, root, root.incrementIdCounter());
        DecisionNode decisionB = new DecisionNode(1, 1, null, null, null, epochB, 
                                                  new PropoagatedEnvironment(targetsPropB, taskB), 
                                                  root.incrementIdCounter());
        chanceB.setChild(decisionB);
        AbsoluteDate epochBPrime = epochB.shiftedBy(2. * 60.);
        KeplerianPropagator kepPropo2 = new KeplerianPropagator(orbit2);
        ObservedObject[] predAndCorrBPrime = 
                TrackingObjective.estimateStateWithOwnExtendedKalman(kepPropo2, epochBPrime, R, obj2, new double[2], sensor);     
        List<ObservedObject> targetsPropBPrime = new ArrayList<ObservedObject>();
        targetsPropBPrime.add(predAndCorrB[1]);
        targetsPropBPrime.add(predAndCorrBPrime[1]);     
        ChanceNode chanceBPrime = 
            new ChanceNode(null, 1, 1, track1, null, decisionB, root.incrementIdCounter());
        DecisionNode decisionBPrime = new DecisionNode(0, 1, null, null, null, epochBPrime, 
                                                       new PropoagatedEnvironment(targetsPropBPrime, taskB), 
                                                       root.incrementIdCounter());
        DecisionNode.setParent(decisionBPrime, chanceBPrime);  
        MultiObjectiveMcts mcts2 = 
            new MultiObjectiveMcts(root, objectives, initDate, epochAPrime, "TDRS Station", 
                                   targetsInit, new ArrayList<ObservedObject>(), sensor);
        mcts2.backpropagate(decisionB, decisionBPrime);

        // Compare
        Assert.assertEquals(1., root.getUtility(), 1e-16);
        Assert.assertEquals(3, root.getNumVisits());
        Assert.assertEquals(1., chanceB.getUtility(), 1e-16);
        Assert.assertEquals(2, chanceB.getNumVisits());
        Assert.assertEquals(1., decisionB.getUtility(), 1e-16);
        Assert.assertEquals(2, decisionB.getNumVisits());

        // Set nodes c
        decisionB.clearChildren();
        AbsoluteDate epochC = epochB.shiftedBy(10. * 60.);
        ChanceNode chanceC = 
            new ChanceNode(null, 1, 1, search1, null, decisionB, root.incrementIdCounter());
        DecisionNode decisionC = new DecisionNode(0, 1, null, null, null, epochC, 
                                                new PropoagatedEnvironment(targetsInit, taskA),
                                                root.incrementIdCounter());
        chanceC.setChild(decisionC);
        AbsoluteDate epochCPrime = epochC.shiftedBy(2. * 60.);
        KeplerianPropagator kepPropo3 = new KeplerianPropagator(orbit2);
        ObservedObject[] predAndCorrCPrime = 
                TrackingObjective.estimateStateWithOwnExtendedKalman(kepPropo3, epochCPrime, R, obj2, new double[2], sensor);
        List<ObservedObject> targetsPropCPrime = new ArrayList<ObservedObject>();
        targetsPropCPrime.add(predAndCorrB[1]);
        targetsPropCPrime.add(predAndCorrCPrime[1]);
        TrackingObjective track2 = new TrackingObjective(targetsPropB, sensor, epochCPrime);
        ChanceNode chanceCPrime = 
            new ChanceNode(null, 1, 1, track2, null, decisionC, root.incrementIdCounter());
        DecisionNode decisionCPrime = new DecisionNode(0, 1, null, null, null, epochCPrime, 
                                                   new PropoagatedEnvironment(targetsPropCPrime, taskA), 
                                                   root.incrementIdCounter());
        DecisionNode.setParent(decisionCPrime, chanceCPrime);
        MultiObjectiveMcts mcts3 = 
            new MultiObjectiveMcts(root, objectives, initDate, epochCPrime, "TDRS Station", 
                                   targetsInit, new ArrayList<ObservedObject>(), sensor);
        mcts3.backpropagate(decisionC, decisionCPrime);

        // Compare
        Assert.assertEquals(1., root.getUtility(), 1e-16);
        Assert.assertEquals(4, root.getNumVisits());
        Assert.assertEquals(1., chanceB.getUtility(), 1e-16);
        Assert.assertEquals(3, chanceB.getNumVisits());
        Assert.assertEquals(1., decisionB.getUtility(), 1e-16);
        Assert.assertEquals(3, decisionB.getNumVisits());
        Assert.assertEquals(2, chanceC.getNumVisits());
        Assert.assertEquals(1., chanceC.getUtility(), 1e-16);
        Assert.assertEquals(2, decisionC.getNumVisits());
        Assert.assertEquals(0., decisionC.getUtility(), 1e-16);

        // Expand branch A 
        AbsoluteDate epochAPrimePrime = epochAPrime.shiftedBy(10. * 60.);
        List<Integer> taskAPrimePrime = new ArrayList<>();
        taskA.add(2);
        taskA.add(0);
        SearchObjective search4 = new SearchObjective(taskAPrimePrime, null, null, 0, null);
        ChanceNode chanceAPrimePrime = 
            new ChanceNode(null, 1, 1, search4, null, decisionAPrime, root.incrementIdCounter());
        DecisionNode decisionAPrimePrime = new DecisionNode(0, 1, null, null, null, 
                                                epochAPrimePrime, 
                                                new PropoagatedEnvironment(targetsPropA, taskAPrimePrime),
                                                root.incrementIdCounter());
        DecisionNode.setParent(decisionAPrimePrime, chanceAPrimePrime);
        MultiObjectiveMcts mcts4 =
            new MultiObjectiveMcts(root, objectives, initDate, epochAPrimePrime, "TDRS Station", 
                                   targetsInit, new ArrayList<ObservedObject>(), sensor);
        mcts4.backpropagate(decisionAPrime, decisionAPrimePrime);
        Assert.assertEquals(0., root.getUtility(), 1e-16);
        Assert.assertEquals(5, root.getNumVisits());
        Assert.assertEquals(0., chanceA.getUtility(), 1e-16);
        Assert.assertEquals(3, chanceA.getNumVisits());
        Assert.assertEquals(-1., decisionA.getUtility(), 1e-16);
        Assert.assertEquals(3, decisionA.getNumVisits());
        Assert.assertEquals(0., chanceAPrime.getUtility(), 1e-16);
        Assert.assertEquals(2, chanceAPrime.getNumVisits());
        Assert.assertEquals(-1., decisionAPrime.getUtility(), 1e-16);
        Assert.assertEquals(2, decisionAPrime.getNumVisits());

   }
}
