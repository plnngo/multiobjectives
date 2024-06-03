package sensortasking.mcts;

import java.io.File;

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
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.models.earth.ReferenceEllipsoid;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.IERSConventions;

/**
 * Hello world!
 *
 */
public class App {
    public static void main( String[] args ){
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
            final StateCovariance covInit = new StateCovariance(covInitMatrix, initialDate, FramesFactory.getTEME(), OrbitType.CARTESIAN, PositionAngle.MEAN);
            final String stmAdditionalName = "stm";
            final MatricesHarvester harvester = tleProp.setupMatricesComputation(stmAdditionalName, null, null);

            final StateCovarianceMatrixProvider provider = new StateCovarianceMatrixProvider("covariance", stmAdditionalName,
                                                                                         harvester, OrbitType.CARTESIAN, PositionAngle.MEAN, covInit);

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
