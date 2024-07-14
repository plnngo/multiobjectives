package sensortasking.mcts;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.DiagonalMatrix;
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
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public class MultiObjectiveMctsTest {

    /** Tree structure stored in a root node. */
    Node root;

    @Before
    public void init() {
        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Build up test decision tree
        root = new DecisionNode(28, 4, null, null, null, new AbsoluteDate(), null);
        root.setId(0);
        
        ChanceNode child1 = new ChanceNode(null, 17, 2, null, null, root);
        ChanceNode child2 = new ChanceNode(null, 8, 2, null, null, root);
        child1.setId(1);
        child2.setId(2);
        root.setChild(child1);
        root.setChild(child2);

        DecisionNode grandchild1 = new DecisionNode(10, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode grandchild2 = new DecisionNode(7, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode grandchild3 = new DecisionNode(3, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode grandchild4 = new DecisionNode(5, 1, null, null, null, new AbsoluteDate(), null);
        grandchild1.setId(3);
        grandchild2.setId(4);
        grandchild3.setId(5);
        grandchild4.setId(6);
        child1.setChild(grandchild1);
        child1.setChild(grandchild2);
        child2.setChild(grandchild3);
        child2.setChild(grandchild4);

        ChanceNode ggchild1 = new ChanceNode(null, 10, 1, null, null, grandchild1);
        ChanceNode ggchild2 = new ChanceNode(null, 7, 1, null, null, grandchild2);
        ggchild1.setId(7);
        ggchild2.setId(8);
        grandchild1.setChild(ggchild1);
        grandchild2.setChild(ggchild2);

        DecisionNode gggchild1 = new DecisionNode(10, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode gggchild2 = new DecisionNode(7, 1, null, null, null, new AbsoluteDate(), null);
        gggchild1.setId(9);
        gggchild2.setId(10);
        ggchild1.setChild(gggchild1);
        ggchild2.setChild(gggchild2);
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
        Node actuallySelected = MultiObjectiveMcts.selectChild(parent);

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
    public void testSelectTdrs() {

        // Epoch
        AbsoluteDate current = new AbsoluteDate(2024, 7, 12, 12, 0, 0., TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = current.shiftedBy(60.*5.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Create list of objects of interest
        TLE tleTdrs05 = new TLE("1 21639U 91054B   24190.31993666 -.00000307  00000-0  00000-0 0  9990", 
                                "2 21639  14.1597 358.9539 0002937 207.3422 133.3222  1.00277803120606");      
        TLE tleTdrs06 = new TLE("1 22314U 93003B   24190.32793498 -.00000302  00000-0  00000-0 0  9994",
                                "2 22314  14.1631   2.2562 0011972 156.4976 200.1996  1.00268889115292");
        TLE tleTdrs12 = new TLE("1 39504U 14004A   24190.25733250 -.00000273  00000-0  00000-0 0  9996", 
                                "2 39504   3.5544   3.9152 0003777 189.8619 144.5768  1.00276604 37168");
        TLE tleTdrs13 = new TLE("1 42915U 17047A   24190.33472274 -.00000099  00000-0  00000-0 0  9997", 
                                "2 42915   3.6233 345.0954 0017520 118.0871 292.6285  1.00272287 25247");

        // Compute state
        TLEPropagator propTdrs05 = TLEPropagator.selectExtrapolator(tleTdrs05);
        TLEPropagator propTdrs06 = TLEPropagator.selectExtrapolator(tleTdrs06);
        TLEPropagator propTdrs12 = TLEPropagator.selectExtrapolator(tleTdrs12);
        TLEPropagator propTdrs13 = TLEPropagator.selectExtrapolator(tleTdrs13);

        SpacecraftState spacecraftTdrs05 = propTdrs05.propagate(current);
        SpacecraftState spacecraftTdrs06 = propTdrs06.propagate(current);
        SpacecraftState spacecraftTdrs12 = propTdrs12.propagate(current);
        SpacecraftState spacecraftTdrs13 = propTdrs13.propagate(current);

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
        // CartesianCovariance stateCovTdrs13 = 
        //     ObservedObject.stateCovToCartesianCov(spacecraftTdrs13.getOrbit(), covEci, j2000);

        // Create list of objects of interest
        ObservedObject tdrs05 = new ObservedObject(tleTdrs05.getSatelliteNumber(), stateTdrs05, stateCovTdrs05, current, j2000);
        ObservedObject tdrs06 = new ObservedObject(tleTdrs06.getSatelliteNumber(), stateTdrs06, stateCovTdrs06, current, j2000);
        ObservedObject tdrs12 = new ObservedObject(tleTdrs12.getSatelliteNumber(), stateTdrs12, stateCovTdrs12, current, j2000);
        //ObservedObject tdrs13 = new ObservedObject(tleTdrs13.getSatelliteNumber(), stateTdrs13, stateCovTdrs13, current, j2000);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(tdrs05);
        ooi.add(tdrs06);
        ooi.add(tdrs12);
        //ooi.add(tdrs13);

         // Ground station
        PVCoordinates pvEcef =  spacecraftTdrs13.getPVCoordinates(ecef);
        double lon = pvEcef.getPosition().getAlpha();
        double lat = pvEcef.getPosition().getDelta();
        System.out.println("Lat " + FastMath.toDegrees(lat));
        System.out.println("Lon " + FastMath.toDegrees(lon));
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
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
                                     initTimeResources, current, ooi);
        MultiObjectiveMcts mctsTracking = 
            new MultiObjectiveMcts(root, objectives, current, endCampaign, topohorizon, ooi, 
                                   new ArrayList<ObservedObject>());
        Node lastLeaf = mctsTracking.select(root);
        //System.out.println(rootUpdated.getUtility());
    }
}
