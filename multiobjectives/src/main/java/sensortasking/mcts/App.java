package sensortasking.mcts;

import java.io.File;
import java.util.Locale;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
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
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
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
            TLE iss = new TLE("1 25544U 98067A   24138.42237498  .00027365  00000-0  46131-3 0  9998",
                            "2 25544  51.6391 107.8732 0003463 176.7237 327.9829 15.51459620453778");

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
            tleProp.addEventDetector(sta1Visi);

            // Propagate from the initial date to the first raising or for the fixed duration
            final SpacecraftState finalState = tleProp.propagate(initialDate.shiftedBy(60.*60.*48));

            System.out.println(" Final state duration from initial date (s) : " + finalState.getDate().durationFrom(initialDate));

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
        }

    }
}
