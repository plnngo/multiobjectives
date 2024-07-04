package sensortasking.mcts;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
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
import org.orekit.estimation.measurements.AngularAzEl;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEConstants;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.analytical.tle.generation.FixedPointTleGenerationAlgorithm;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AbsolutePVCoordinates;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class TrackingObjectiveTest {

    /** Topocentric horizon frame.*/
    TopocentricFrame topoHorizon;

    /** Test object. */
    ObservedObject singleTestCase = null;

    @Before
    public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Location at United States Air Force Academy according to Vallado Example 4-1
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),   // Geodetic latitude
                                              FastMath.toRadians(-104.883),   // Longitude
                                              2194.56);              // in [m]

        // Set up topocentric horizon frame
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               earthFrame);
        this.topoHorizon = new TopocentricFrame(earth, pos, "United States Air Force Academy");
    }

    private void generateTestObject() {

        // Create frames
        Frame eci = FramesFactory.getGCRF();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame teme = FramesFactory.getTEME();

        // Set up input data from Vallado
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        PVCoordinates pvEci = 
            new PVCoordinates(new Vector3D(-605792.21660, -5870229.51108, 3493053.19896),
                              new Vector3D(-1568.25429, -3702.34891, -6479.48395));

        // Transform from ECI to TEME and ECEF
        Transform eciToTeme = eci.getTransformTo(teme, date);
        PVCoordinates pvTeme = eciToTeme.transformPVCoordinates(pvEci);
        //AbsolutePVCoordinates pv = 
        //    new AbsolutePVCoordinates(eci, new TimeStampedPVCoordinates(date, pvTeme));
        
        //CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        KeplerianOrbit kep = new KeplerianOrbit(pvTeme, teme, date, TLEConstants.MU);
        SpacecraftState scStateTeme = new SpacecraftState(kep);
        StateVector stateTeme = ObservedObject.spacecraftStateToStateVector(scStateTeme, teme);

        // Set covariance matrix
        double[][] covArray = new double[6][6];
        covArray[0] = 
            new double[]{0.9999945079552018, 0.009999542228922726, 0.009997709967860641, 
                         9.994507954236584E-5, 9.999542225752099E-5, 9.997710067646445E-5};
        covArray[1] =
            new double[]{0.009999542228922725, 1.0000045790384262, 0.010002745854447466, 
                         9.999542227959272E-5, 1.0004579035257533E-4, 1.0002746172251207E-4};
        covArray[2] =
            new double[]{0.00999770996786064, 0.010002745854447466, 1.0000009130063703, 
                         9.997709871242466E-5, 1.0002745537610997E-4, 1.000091301050587E-4};
        covArray[3] =
            new double[]{9.994507954236582E-5, 9.999542227959272E-5, 9.997709871242466E-5, 
                         9.99450795327065E-7, 9.999542224788645E-7, 9.997709971028265E-7};
        covArray[4] =
            new double[]{9.999542225752099E-5, 1.0004579035257532E-4, 1.0002745537610997E-4, 
                         9.999542224788643E-7, 1.0004579032088274E-6, 1.0002745855414732E-6};
        covArray[5] = 
            new double[]{9.997710067646443E-5, 1.0002746172251205E-4, 1.0000913010505871E-4, 
                         9.99770997102827E-7, 1.0002745855414737E-6, 1.000091301464106E-6};
        RealMatrix covMatrix = MatrixUtils.createRealMatrix(covArray);
        StateCovariance covTeme = new StateCovariance(covMatrix, date, teme, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        
        CartesianCovariance stateCovTeme = 
            ObservedObject.stateCovToCartesianCov(kep, covTeme, teme);

        // Generate list of object of interest OOI
        //LeastSquaresTleGenerationAlgorithm converter = new LeastSquaresTleGenerationAlgorithm();
        TLE testTle = new TLE("1 25544U 98067A   21035.14486477  .00001026  00000-0  26816-4 0  9998",
                              "2 25544  51.6455 280.7636 0002243 335.6496 186.1723 15.48938788267977");
        double n = FastMath.sqrt(TLEConstants.MU/FastMath.pow(kep.getA(), 3));
        TLE tle = new TLE(testTle.getSatelliteNumber(), testTle.getClassification(), 
                            testTle.getLaunchYear(), testTle.getLaunchNumber(), testTle.getLaunchPiece(), 
                            testTle.getEphemerisType(), testTle.getElementNumber(), date, n, 
                            testTle.getMeanMotionFirstDerivative(), testTle.getMeanMotionSecondDerivative(), 
                            kep.getE(), kep.getI(), kep.getPerigeeArgument(), 
                            kep.getRightAscensionOfAscendingNode(), kep.getMeanAnomaly(), 
                            testTle.getRevolutionNumberAtEpoch(), testTle.getBStar(), 
                            TimeScalesFactory.getUTC());
        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(scStateTeme, tle);
        System.out.println(pseudoTle);
        this.singleTestCase = new ObservedObject(123, stateTeme , stateCovTeme, teme, pseudoTle);

        // Retrieve coordinates of ground station
        Transform eciToEcef = eci.getTransformTo(ecef, date);
        PVCoordinates pvEcef =  eciToEcef.transformPVCoordinates(pvEci);
        double lon = pvEcef.getPosition().getAlpha();
        double lat = pvEcef.getPosition().getDelta();
        // System.out.println("Lat " + FastMath.toDegrees(lat));
        // System.out.println("Lon " + FastMath.toDegrees(lon));

        GeodeticPoint pos = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        this.topoHorizon = new TopocentricFrame(earth, pos, "Generic Station");

    }

    /**
     * Test {@link TrackingObjective#setMicroAction(AbsoluteDate)}
     * using test data from "Covariance Transformations for Satellite Flight Dynamics Operations"
     * @throws IOException
     */
    @Test
    public void testSetMicroAction() throws IOException {

        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        generateTestObject();
        
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(this.singleTestCase);

        // Call test function 
        //TODO: work on topocentric inertial frame
        TrackingObjective trackTask = new TrackingObjective(ooi, this.topoHorizon, null);
        AngularDirection pointing = trackTask.setMicroAction(date);
        System.out.println("Azimuth: " + FastMath.toDegrees(pointing.getAngle1()));
        System.out.println("Elevation: " + FastMath.toDegrees(pointing.getAngle2()));


        // Load TLEs from file
        /* String workingDir = System.getProperty("user.dir");
        String fakeTleDataDir = "\\src\\test\\java\\resources\\test-data\\noaa15fakeTles.tle";
        File testData = new File(workingDir + fakeTleDataDir);
        BufferedReader reader = new BufferedReader(new FileReader(testData));
        List<TLE> tleSeries = new ArrayList<TLE>();
        String current = "";

        // Generate list of observed objects
        List<ObservedObject> toTrack = new ArrayList<ObservedObject>();
        while((current = reader.readLine())!=null){
            TLE orbit = new TLE(current, reader.readLine());
            TLEPropagator prop = TLEPropagator.selectExtrapolator(orbit);
            SpacecraftState stateOrekit = prop.propagate(orbit.getDate());
            StateVector state = ObservedObject.spacecraftStateToStateVector(stateOrekit, topoHorizon);

            RealMatrix covMatrix = 
                MatrixUtils.createRealDiagonalMatrix(new double[]{100 * 1e3, 100 * 1e3, 100 * 1e3, 
                                                                  0.1, 0.1, 0.1});
            StateCovariance stateCov = 
                new StateCovariance(covMatrix, orbit.getDate(), FramesFactory.getTEME(), 
                                    OrbitType.CARTESIAN, PositionAngle.MEAN);
            ObservedObject.stateCovToCartesianCov(stateOrekit.getOrbit(), stateCov, topoHorizon);
            


            ObservedObject obj = new ObservedObject(orbit.getSatelliteNumber(), state, null, topoHorizon, orbit);
            tleSeries.add(new TLE(current, reader.readLine()));
        }
        reader.close(); */

    }

    @Test
    public void testSetMicroAction2() {

        // Epoch
        AbsoluteDate date = new AbsoluteDate(2024, 6, 26, 0, 0, 0., TimeScalesFactory.getUTC());

        // Frame
        Frame teme = FramesFactory.getTEME();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Set up ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(5.),   // Geodetic latitude
                                              FastMath.toRadians(-172.),   // Longitude
                                              0.);              // in [m]

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, pos, "Station to observe TDRS satellite");


        // Create list of objects of interest
        TLE tleTdrs10 = new TLE("1 27566U 02055A   24176.82688309  .00000089  00000-0  00000-0 0  9999", 
                                "2 27566   9.5777  45.3560 0006120 347.9755   6.7068  1.00267284 78984");
        TLE tleTdrs11 = new TLE("1 39070U 13004A   24176.84939448  .00000087  00000-0  00000-0 0  9991", 
                                "2 39070   2.7492  11.9730 0005934  54.3648 338.6204  1.00277276 39456");

        // Compute state
        TLEPropagator propTdrs10 = TLEPropagator.selectExtrapolator(tleTdrs10);
        TLEPropagator propTdrs11 = TLEPropagator.selectExtrapolator(tleTdrs11);

        SpacecraftState spacraftTdrs10 = propTdrs10.propagate(date);
        SpacecraftState spacraftTdrs11 = propTdrs11.propagate(date);

        StateVector stateTdrs10 = ObservedObject.spacecraftStateToStateVector(spacraftTdrs10, teme);
        StateVector stateTdrs11 = ObservedObject.spacecraftStateToStateVector(spacraftTdrs11, teme);

        // Set covariances
        double[] covDiag = new double[]{100., 100., 100., 1., 1., 1.};
        RealMatrix covMatrix = MatrixUtils.createRealDiagonalMatrix(covDiag);
        StateCovariance covTeme = new StateCovariance(covMatrix, date, teme, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        
        CartesianCovariance stateCovTdrs10 = 
            ObservedObject.stateCovToCartesianCov(spacraftTdrs10.getOrbit(), covTeme, teme);
        CartesianCovariance stateCovTdrs11 = 
            ObservedObject.stateCovToCartesianCov(spacraftTdrs11.getOrbit(), covTeme, teme);

        // Create list of objects of interest
        ObservedObject tdrs10 = new ObservedObject(tleTdrs10.getSatelliteNumber(), stateTdrs10, stateCovTdrs10, teme, tleTdrs10);
        ObservedObject tdrs11 = new ObservedObject(tleTdrs11.getSatelliteNumber(), stateTdrs11, stateCovTdrs11, teme, tleTdrs11);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(tdrs10);
        ooi.add(tdrs11);

/*         // Ground station
        Transform temeToEcef = teme.getTransformTo(ecef, date);
        PVCoordinates pvTeme = 
            new PVCoordinates(stateTdrs11.getPositionVector(), stateTdrs11.getVelocityVector());
        PVCoordinates pvEcef =  temeToEcef.transformPVCoordinates(pvTeme);
        double lon = pvEcef.getPosition().getAlpha();
        double lat = pvEcef.getPosition().getDelta();
        System.out.println("Lat " + FastMath.toDegrees(lat));
        System.out.println("Lon " + FastMath.toDegrees(lon)); */

        TrackingObjective tracking = new TrackingObjective(ooi, topoHorizon, null);
        AngularDirection pointing = tracking.setMicroAction(date);
        System.out.println("Azimuth: " +FastMath.toDegrees(pointing.getAngle1()));
        System.out.println("Elevation: " +FastMath.toDegrees(pointing.getAngle2()));
    }

    @Test
    public void testSetMicroActionKeplerian(){
        // Frame
        Frame eci = FramesFactory.getEME2000();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Date
        AbsoluteDate current = AbsoluteDate.J2000_EPOCH.shiftedBy(584.);
        AbsoluteDate target = current.shiftedBy(100.);

        // Definition of initial conditions with position and velocity
        Vector3D position = new Vector3D(7.0e6, 1.0e6, 4.0e6);
        Vector3D velocity = new Vector3D(-500.0, 8000.0, 1000.0);
        PVCoordinates pvInit = new PVCoordinates(position, velocity);
        KeplerianOrbit kep = new KeplerianOrbit(pvInit, eci, current, Constants.WGS84_EARTH_MU);
        SpacecraftState scStateEci = new SpacecraftState(kep);
        StateVector stateEci = ObservedObject.spacecraftStateToStateVector(scStateEci, eci);

        // Initial covariance
        RealMatrix covInitMatrix = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                              0.1, 0.1, 0.1});
        StateCovariance covInit = 
            new StateCovariance(covInitMatrix, current, eci, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN);
        CartesianCovariance stateCovEci = 
            ObservedObject.stateCovToCartesianCov(kep, covInit, eci);

        // Propagate to see where object is
        KeplerianPropagator extrapolator = new KeplerianPropagator(kep);
        SpacecraftState finalOrbit = extrapolator.propagate(target);
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
        Transform eciToTopo = new Transform(target, coordinatesStationEci.negate());
        Frame topoCentric = new Frame(eci, eciToTopo, "Topocentric", true);

        ObservedObject singleKeplerianTestCase = 
            new ObservedObject(987, stateEci, stateCovEci, current, eci);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(singleKeplerianTestCase);
        TrackingObjective track = new TrackingObjective(ooi, topoHorizon, topoCentric);
        AngularDirection micro = track.setMicroAction(current);
        Assert.assertEquals(AngleType.RADEC, micro.getAngleType());
        Assert.assertEquals(14.535587965895044, FastMath.toDegrees(micro.getAngle1()), 1e-12);
        Assert.assertEquals(30.295623596303162, FastMath.toDegrees(micro.getAngle2()), 1e-12);
    }

    /**
     * Test {@link TrackingObjective#computeKullbackLeiblerDivergence(ObservedObject, 
     * ObservedObject)}.
     */
    @Test
    public void testComputeKullbackLeiblerDivergence() {
        Frame eci = FramesFactory.getGCRF();
        AbsoluteDate date = new AbsoluteDate();

        // Define posterior object
        StateVector postState = new StateVector();
        postState.setX(1);
        postState.setY(2);
        postState.setZ(3);
        postState.setXdot(4);
        postState.setYdot(5);
        postState.setZdot(6);
        CartesianOrbit orbit = 
            new CartesianOrbit(new PVCoordinates(postState.getPositionVector(), 
                                                 postState.getVelocityVector()), 
                               eci, date, Constants.WGS84_EARTH_MU);
        CartesianCovariance postCov = new CartesianCovariance(null);
        double[][] covArray = new double[6][6];
        covArray[0] = new double[]{1,8,2,3,2,7};
        covArray[1] = new double[]{8,0,2,2,1,8};
        covArray[2] = new double[]{2,2,-6,1,6,9};
        covArray[3] = new double[]{3,2,1,1,8,1};
        covArray[4] = new double[]{2,1,6,8,6,2};
        covArray[5] = new double[]{7,8,9,1,2,3};
        StateCovariance covStateCov = 
            new StateCovariance(MatrixUtils.createRealMatrix(covArray), date, eci, 
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        postCov = ObservedObject.stateCovToCartesianCov(orbit , covStateCov, eci);
        ObservedObject post = new ObservedObject(0, postState, postCov, date, eci);

        // Define posterior object
        StateVector priorState = new StateVector();
        priorState.setX(2);
        priorState.setY(3);
        priorState.setZ(1);
        priorState.setXdot(1);
        priorState.setYdot(4);
        priorState.setZdot(7);
        CartesianOrbit orbitPost = 
            new CartesianOrbit(new PVCoordinates(priorState.getPositionVector(), 
                               priorState.getVelocityVector()), 
                               eci, date, Constants.WGS84_EARTH_MU);
        CartesianCovariance priorCov = new CartesianCovariance(null);
        double[][] covArrayPost = new double[6][6];
        covArrayPost[0] = new double[]{1,1,2,1,5,3};
        covArrayPost[1] = new double[]{1,2,1,2,2,2};
        covArrayPost[2] = new double[]{2,1,5,1,1,1};
        covArrayPost[3] = new double[]{1,2,1,1,7,7};
        covArrayPost[4] = new double[]{5,2,1,7,1,1};
        covArrayPost[5] = new double[]{3,2,1,7,1,0};
        StateCovariance covPriorStateCov = 
            new StateCovariance(MatrixUtils.createRealMatrix(covArrayPost), date, eci, 
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        priorCov = ObservedObject.stateCovToCartesianCov(orbitPost , covPriorStateCov, eci);
        ObservedObject prior = new ObservedObject(1, priorState, priorCov, date, eci);

        double kl = TrackingObjective.computeKullbackLeiblerDivergence(prior, post);
        Assert.assertEquals(228.2739649346909, kl, 1e-12);
        Assert.assertEquals(0., TrackingObjective.computeKullbackLeiblerDivergence(prior, prior), 
                            1e-12);
        Assert.assertEquals(0., TrackingObjective.computeKullbackLeiblerDivergence(post, post), 
                            1e-12);
    }

    @Test
    public void testEstimateStateWithKalman() {
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        generateTestObject();
        
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(this.singleTestCase);

        // Call test function 
        TrackingObjective trackTask = new TrackingObjective(ooi, this.topoHorizon, null);

        // Compute sensor pointing
        Entry<SpacecraftState, StateCovariance> stateAndCov = 
                        TrackingObjective.propagateStateAndCovariance(this.singleTestCase, date);
        AngularDirection meas = 
            TrackingObjective.transformStateToPointing(stateAndCov.getKey(), this.topoHorizon);

        /* double[] angles = new double[]{FastMath.toRadians(90.0000000163951), 
                                       FastMath.toRadians(87.78861644886392)};
        AngularDirection meas = new AngularDirection(topoHorizon, angles, AngleType.AZEL); */
        meas.setDate(date);
        AngularAzEl measOrekit = TrackingObjective.transformAngularAzEl2OrekitMeasurements(meas, topoHorizon);
        List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
        orekitAzElMeas.add(measOrekit);
        
        ObservedObject[] results = trackTask.estimateStateWithKalman(orekitAzElMeas, singleTestCase);
        for(int i=0; i<results.length; i++) {
            System.out.println("Pos: " + results[i].getState().getPositionVector() + "Vel: " + results[i].getState().getVelocityVector());
        }
        
        
    }

    @Test
    public void testGenerateMeasurements() {
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        generateTestObject();
        
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(this.singleTestCase);

        // Call test function 
        TrackingObjective trackTask = new TrackingObjective(ooi, this.topoHorizon, null);
        
        // Compute sensor pointing
        Entry<SpacecraftState, StateCovariance> stateAndCov = 
            TrackingObjective.propagateStateAndCovariance(this.singleTestCase, date);
        AngularDirection pointing = 
            TrackingObjective.transformStateToPointing(stateAndCov.getKey(), this.topoHorizon);
        List<AngularDirection> measurements = 
            trackTask.generateMeasurements(singleTestCase, pointing, 7., 8., date);
        Assert.assertEquals(1, measurements.size());
        AngularDirection meas = measurements.get(0);
        Assert.assertEquals(FastMath.toDegrees(pointing.getAngle1()), 
                            FastMath.toDegrees(meas.getAngle1()), 1e-16);
        Assert.assertEquals(FastMath.toDegrees(pointing.getAngle2()), 
                            FastMath.toDegrees(meas.getAngle2()), 1e-16);
    }

    /**
     * Propagate initial covariance using SGP4 to initial epoch.
     */
    @Test
    public void testPropagateCovariance() {
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        generateTestObject();
        Entry<SpacecraftState, StateCovariance> propagated = TrackingObjective.propagateStateAndCovariance(this.singleTestCase, date);
        RealMatrix actualCov = propagated.getValue().getMatrix();
        RealMatrix expectedCov = this.singleTestCase.getCovariance().getCovarianceMatrix();

        // Compare covariance
        for (int row=0; row<expectedCov.getRowDimension(); row++) {
            double[] expectedRowVec = expectedCov.getRow(row);
            double[] actualRowVec = actualCov.getRow(row);
            for(int col=0; col<expectedCov.getColumnDimension(); col++) {
                Assert.assertEquals(expectedRowVec[col], actualRowVec[col], 1e-16);
            }
        }

        // Compare state
        double[] actualPos = propagated.getKey().getPVCoordinates().getPosition().toArray();
        double[] actualVel = propagated.getKey().getPVCoordinates().getVelocity().toArray();
        double[] expectedPos = this.singleTestCase.getState().getPositionVector().toArray();
        double[] expectedVel = this.singleTestCase.getState().getVelocityVector().toArray();
        
        for (int dim=0; dim<3; dim++) {
            Assert.assertEquals(expectedPos[dim], actualPos[dim], 1e-4);
            Assert.assertEquals(expectedVel[dim], actualVel[dim], 1e-7);
        }
    }

    /**
     * Test {@link TrackingObjective#transformStateToAzEl(SpacecraftState)} using the  
     * reference coordinates from Vallado's Example 4-1 in "Fundamentals of Astrodynamics and 
     * Applications". The input is first converted into TEME since 
     * {@link TrackingObjective#transformStateToAzEl(SpacecraftState)} is usually called on states
     * that are defined in TEME.
     */
    @Test
    public void testTransformStateToAzEl() {
        // Frames
        Frame eci = FramesFactory.getGCRF();
        Frame teme = FramesFactory.getTEME();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                            Constants.WGS84_EARTH_FLATTENING,
                                            ecef);
        GeodeticPoint posStation = new GeodeticPoint(FastMath.toRadians(39.007),     // Geodetic latitude
                                                     FastMath.toRadians(-104.883),     // Longitude
                                            2194.56);                         // Altitude in [m]
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, posStation, "Topocentric Horizon");

        // Input
        AbsoluteDate date = 
            new AbsoluteDate(1994, 5, 14, 13, 11, 20.59856, TimeScalesFactory.getUTC());
        Vector3D pos = new Vector3D(5036.736529 * 1e3, -10806.660797 * 1e3, -4534.633784 * 1e3);
        Vector3D vel = new Vector3D(2.6843855 * 1e3, -5.7595920 * 1e3, -2.4168093* 1e3);

        Transform eciToTeme = eci.getTransformTo(teme, date);
        PVCoordinates pvTeme =  eciToTeme.transformPVCoordinates(new PVCoordinates(pos, vel));
        AbsolutePVCoordinates pv = 
            new AbsolutePVCoordinates(teme, new TimeStampedPVCoordinates(date, pvTeme));
        SpacecraftState scState = new SpacecraftState(pv);
        StateVector stateVec = ObservedObject.spacecraftStateToStateVector(scState, teme);
        ObservedObject obj = new ObservedObject(234, stateVec, null, date, teme);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(obj);

        // Expected data wrt Vallado's definition of topocentric horizon frame
        double[] expectedVallado = new double[]{FastMath.toRadians(210.8777747), 
                                                FastMath.toRadians(-5.9409535)};
        double[] expectedOrekit = new double[]{-(expectedVallado[0]-2*FastMath.PI) + FastMath.PI/2,
                                            expectedVallado[1]};
        AngularDirection actualOrekit = 
            TrackingObjective.transformStateToPointing(scState, topoHorizon);
        // Compare
        double tolerance = 1e-3;
        Assert.assertEquals(FastMath.toDegrees(expectedOrekit[0]), 
                            FastMath.toDegrees(actualOrekit.getAngle1()), tolerance);
        Assert.assertEquals(FastMath.toDegrees(expectedOrekit[1]), 
                            FastMath.toDegrees(actualOrekit.getAngle2()), tolerance);
    }

    /**
     * Test {@link TrackingObjective#transformAngularAzEl2OrekitMeasurements(AngularDirection, 
     * TopocentricFrame)}. 
     */
    @Test
    public void testTransformAngularAzEl2OrekitMeasurements() {

        AngularDirection test = 
            new AngularDirection(topoHorizon, new double[]{FastMath.PI/2, FastMath.PI/2}, 
                                 AngleType.AZEL);
        AngularAzEl orekitAzEl = TrackingObjective.transformAngularAzEl2OrekitMeasurements(test, this.topoHorizon);
        double[] actual = orekitAzEl.getObservedValue();
        Assert.assertEquals(test.getAngle1(), actual[0], 1e-16);
        Assert.assertEquals(test.getAngle2(), actual[1], 1e-16);

        test = 
            new AngularDirection(topoHorizon, new double[]{0., FastMath.toRadians(78)}, 
                                 AngleType.AZEL);
        orekitAzEl = TrackingObjective.transformAngularAzEl2OrekitMeasurements(test, this.topoHorizon);
        actual = orekitAzEl.getObservedValue();
        Assert.assertEquals(test.getAngle1(), actual[0], 1e-16);
        Assert.assertEquals(test.getAngle2(), actual[1], 1e-16);
    }
}
