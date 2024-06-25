package sensortasking.mcts;

import java.io.File;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.events.Action;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.models.earth.ReferenceEllipsoid;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEConstants;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.analytical.tle.generation.FixedPointTleGenerationAlgorithm;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

/**
 * Hello world!
 *
 */
public class App {
    public static void main( String[] args ){

        // Load orekit data
        String orekitDataDir = "multiobjectives\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        //App.testPropo2();


    }

    public static void propagateCovarianceOrekitExample() {
        Frame teme = FramesFactory.getTEME();
        double MU = 3.986004415 * 1e14;     // in m^3/s^(-2)

        // Initiate scenario
        AbsoluteDate date = new AbsoluteDate("2016-02-13T16:00:00.0", TimeScalesFactory.getUTC());
        Vector3D pos = new Vector3D(7526.993581890527 * 1e3, 
                                    -9646.310100269711 * 1e3, 
                                    1464.1104928112086 * 1e3);
        Vector3D vel = new Vector3D(3.0337945609969803 * 1e3, 
                                    1.715265069098717 * 1e3, 
                                    -4.447658745923896 * 1e3);
        PVCoordinates pv = new PVCoordinates(pos, vel);
        CartesianOrbit orbit = new CartesianOrbit(pv, teme, date, MU);
        final SpacecraftState stateInit = new SpacecraftState(orbit);


        /* // Propagate from start of window to last measurement epoch without measurement update
        TLEPropagator prop = TLEPropagator.selectExtrapolator(candidate.getPseudoTle());

        // Covariance
        final RealMatrix covInitMatrix = candidate.getCovariance().getCovarianceMatrix();
        final StateCovariance covInit = 
            new StateCovariance(covInitMatrix, candidate.getEpoch(), candidate.getFrame(), 
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = prop.setupMatricesComputation(stmAdditionalName, null, null);

        final StateCovarianceMatrixProvider provider = 
            new StateCovarianceMatrixProvider("cov", stmAdditionalName, harvester, covInit);

        prop.addAdditionalStateProvider(provider);

        final SpacecraftState stateEndNotUpdated = prop.propagate(lastMeasEpoch);
        StateVector stateVecEndNotUpdate = ObservedObject.spacecraftStateToStateVector(stateEndNotUpdated, candidate.getFrame());
        RealMatrix covProp = provider.getStateCovariance(stateEndNotUpdated).getMatrix();
        StateCovariance stateCovEndNotUpdated = 
            new StateCovariance(covProp, lastMeasEpoch, stateEndNotUpdated.getFrame(),       // TODO: check frame
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovNotEndUpdated = 
            ObservedObject.stateCovToCartesianCov(stateEndNotUpdated.getOrbit(), 
                                                  stateCovEndNotUpdated, candidate.getFrame());

        // State at the end of tasking without measurement updates
        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(stateEndNotUpdated, 
                                                                        candidate.getPseudoTle());
        output[1] = new ObservedObject(candidate.getId() + 2, stateVecEndNotUpdate, cartCovNotEndUpdated, 
                                       stationFrame, pseudoTle); */

    }

    public void testFrameConversion() {
        // Load orekit data
        String orekitDataDir = "multiobjectives\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

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
        System.out.println("State in TEME using Transform");
        System.out.println(pvTeme.toString());

        CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        SpacecraftState scStateTeme = new SpacecraftState(orbit);
        StateVector stateTeme = ObservedObject.spacecraftStateToStateVector(scStateTeme, teme);
        System.out.println("State in TEME using spacecraftStateToStateVector");
        System.out.println(stateTeme.getPositionVector());
        System.out.println(stateTeme.getVelocityVector());

        TLE testTle = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");
        //LeastSquaresTleGenerationAlgorithm converter = new LeastSquaresTleGenerationAlgorithm();
        //TLE pseudoTle = TLE.stateToTLE(scStateTeme, testTle, converter);   //TODO: check if SGP4 model is applied or if the state vector is still the same
        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(scStateTeme, testTle);

 /*        // Retrieve semi major axis from TLE
        double n = pseudoTle.getMeanMotion();
        double a = FastMath.pow(TLEConstants.MU, 1./3.)/(FastMath.pow(n, 2./3.));

        KeplerianOrbit kepOrbitFromTle = 
            new KeplerianOrbit(a, pseudoTle.getE(), pseudoTle.getI(), 
                               pseudoTle.getPerigeeArgument(), pseudoTle.getRaan(), 
                               pseudoTle.getMeanAnomaly(), PositionAngleType.MEAN, teme, date, 
                               TLEConstants.MU);
        System.out.println("State in TEME using stateToTLE");
        System.out.println(kepOrbitFromTle.getPVCoordinates().getPosition());
        System.out.println(kepOrbitFromTle.getPVCoordinates().getVelocity()); */

        //SGP4 Propagator 
        TLEPropagator tleProp = TLEPropagator.selectExtrapolator(pseudoTle);
        SpacecraftState stateProp = tleProp.propagate(date);
        System.out.println("State in TEME after propagation");
        System.out.println(stateProp.getPVCoordinates().getPosition());
        System.out.println(stateProp.getPVCoordinates().getVelocity());
        System.out.println("State in TEME before propagation");
        System.out.println(tleProp.getInitialState().getPVCoordinates().getPosition());
        System.out.println(tleProp.getInitialState().getPVCoordinates().getVelocity());
    }

    public static void testPropo() {
        try {

            // Test TLE
            TLE iss = new TLE("1 25544U 98067A   24142.35003124  .00025335  00000-0  42455-3 0  9992",
                            "2 25544  51.6384  88.3676 0003163 192.6310 305.1222 15.51672766454387");

            //  Initial state definition : date, orbit
            final TimeScale utc = TimeScalesFactory.getUTC();
            
            final AbsoluteDate initialDate = iss.getDate();
/*             final double mu =  3.986004415e+14; // gravitation coefficient
            final Frame inertialFrame = FramesFactory.getEME2000(); // inertial frame for orbit definition
            final Vector3D position  = new Vector3D(-6142438.668, 3492467.560, -25767.25680);
            final Vector3D velocity  = new Vector3D(505.8479685, 942.7809215, 7435.922231);
            final PVCoordinates pvCoordinates = new PVCoordinates(position, velocity);
            final Orbit initialOrbit = new KeplerianOrbit(pvCoordinates, inertialFrame, initialDate, mu); */

            // Propagator : consider a simple Keplerian motion (could be more elaborate)
            //final Propagator kepler = new KeplerianPropagator(initialOrbit);
            TLEPropagator tleProp = TLEPropagator.selectExtrapolator(iss);

            // Earth and frame
            final Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
            final BodyShape earth = ReferenceEllipsoid.getWgs84(ecef);

            // Station
            final double longitude = FastMath.toRadians(-104.883);
            final double latitude  = FastMath.toRadians(39.007);
            final double altitude  = 2194.56;
            final GeodeticPoint station1 = new GeodeticPoint(latitude, longitude, altitude);
            final TopocentricFrame sta1Frame = new TopocentricFrame(earth, station1, "station1");

            // Covariance
            final RealMatrix covInitMatrix = new DiagonalMatrix(new double[]{10., 10., 10., 100., 100., 100.});
            final StateCovariance covInit = new StateCovariance(covInitMatrix, initialDate, FramesFactory.getTEME(), OrbitType.CARTESIAN, PositionAngleType.MEAN);
            final String stmAdditionalName = "stm";
            final MatricesHarvester harvester = tleProp.setupMatricesComputation(stmAdditionalName, null, null);

            final StateCovarianceMatrixProvider provider = 
                new StateCovarianceMatrixProvider("cov", stmAdditionalName, harvester, covInit);

            tleProp.addAdditionalStateProvider(provider);

            // Event definition
            final double maxcheck  = 60.0;
            final double threshold =  0.001;
            final double elevation = FastMath.toRadians(5.0);
            
            final EventDetector sta1Visi =
                    new ElevationDetector(maxcheck, threshold, sta1Frame).
                    withConstantElevation(elevation).
                    withHandler((s, d, increasing) -> {
                        System.out.println(" Visibility on " +
                                            ((ElevationDetector) d).getTopocentricFrame().getName() +
                                           (increasing ? " begins at " : " ends at ") +
                                           s.getDate().toStringWithoutUtcOffset(utc, 3));
                        
                        return increasing ? Action.CONTINUE : Action.STOP;
                    });

            // Add event to be detected
            final EventsLogger logger = new EventsLogger();
            tleProp.addEventDetector(logger.monitorDetector(sta1Visi));

            // Propagate from the initial date to the first raising or for the fixed duration
            AbsoluteDate targetEpoch = initialDate.shiftedBy(60.*60.*24);
            targetEpoch = new AbsoluteDate(2024, 5, 21, 21, 56, 0, utc);
            final SpacecraftState finalState = tleProp.propagate(targetEpoch);

            System.out.println(" Final state duration from initial date (s) : " + finalState.getDate().durationFrom(initialDate));
            

             for (LoggedEvent le : logger.getLoggedEvents()) {
                if (le.isIncreasing()) {
                    System.out.println("Enter FOR " + le.getDate());
                } else {
                    System.out.println("Exit FOR " + le.getDate());;
                }
                System.out.println("Propagate the covariance to " + le.getDate().toString() + " ...");
                final RealMatrix covProp = provider.getStateCovariance(le.getState()).getMatrix();
                System.out.println(covProp.toString());
            }
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
        }
    }

    public static void testPropo2() {
        
        // Create frames
        Frame eci = FramesFactory.getGCRF();
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
        //CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        
        KeplerianOrbit kep = new KeplerianOrbit(pvTeme, teme, date, TLEConstants.MU);
        SpacecraftState scStateTeme = new SpacecraftState(kep);
        System.out.println(scStateTeme.getPVCoordinates());
        final TLE tleISS = new TLE("1 25544U 98067A   21035.14486477  .00001026  00000-0  26816-4 0  9998",
                                "2 25544  51.6455 280.7636 0002243 335.6496 186.1723 15.48938788267977");
        double n = FastMath.sqrt(TLEConstants.MU/FastMath.pow(kep.getA(), 3));
        TLE tle = new TLE(tleISS.getSatelliteNumber(), tleISS.getClassification(), 
                            tleISS.getLaunchYear(), tleISS.getLaunchNumber(), tleISS.getLaunchPiece(), 
                            tleISS.getEphemerisType(), tleISS.getElementNumber(), date, n, 
                            tleISS.getMeanMotionFirstDerivative(), tleISS.getMeanMotionSecondDerivative(), 
                            kep.getE(), kep.getI(), kep.getPerigeeArgument(), 
                            kep.getRightAscensionOfAscendingNode(), kep.getMeanAnomaly(), 
                            tleISS.getRevolutionNumberAtEpoch(), tleISS.getBStar(), 
                            TimeScalesFactory.getUTC());

        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(scStateTeme, tle);
        //LeastSquaresTleGenerationAlgorithm converter = new LeastSquaresTleGenerationAlgorithm();
        //TLE pseudoTle = TLE.stateToTLE(scStateTeme, tle, converter);
        TLEPropagator tleProp = TLEPropagator.selectExtrapolator(pseudoTle);
        //System.out.println(tle.toString());
        System.out.println(pseudoTle.toString());

        // Covariance
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
        
        // final RealMatrix covInitMatrix = new DiagonalMatrix(new double[]{10., 10., 10., 100., 100., 100.});
        // final StateCovariance covInit = new StateCovariance(covInitMatrix, date, teme, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "STM";
        final MatricesHarvester harvester = tleProp.setupMatricesComputation(stmAdditionalName, null, null);

        final StateCovarianceMatrixProvider provider = 
            new StateCovarianceMatrixProvider("cartCov", stmAdditionalName, harvester, covTeme);

        tleProp.addAdditionalStateProvider(provider);
        SpacecraftState s = tleProp.propagate(date.shiftedBy(0.));
        StateVector vec = ObservedObject.spacecraftStateToStateVector(s, teme);
        System.out.println(vec.getPositionVector());
        System.out.println(vec.getVelocityVector());
        StateCovariance covProp = provider.getStateCovariance(s);
        RealMatrix covPropMatrix = covProp.getMatrix();
        for (int row=0; row<covPropMatrix.getRowDimension(); row++) {
            double[] rowVec = covPropMatrix.getRow(row);
            for(int col=0; col<covPropMatrix.getColumnDimension(); col++) {
                System.out.print(rowVec[col] + " ");
            }
            System.out.println();
        }
    }
}
