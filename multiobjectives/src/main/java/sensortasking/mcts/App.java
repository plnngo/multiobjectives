package sensortasking.mcts;

import java.io.File;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.DiagonalMatrix;
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

    public void testPropo() {
        try {

            // Load orekit data
            String orekitDataDir = "multiobjectives\\src\\test\\java\\resources\\orekit-data";
            File orekitData = new File(orekitDataDir);
            DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
            manager.addProvider(new DirectoryCrawler(orekitData));

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
}
