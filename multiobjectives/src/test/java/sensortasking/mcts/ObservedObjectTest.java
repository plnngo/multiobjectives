package sensortasking.mcts;

import java.io.File;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
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
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AbsolutePVCoordinates;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class ObservedObjectTest {
    @Before
    public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }

    /**
     * Test {@link ObservedObject#spacecraftStateToStateVector(SpacecraftState)} using the  
     * reference coordinates from Vallado's Example 4-1 in "Fundamentals of Astrodynamics and 
     * Applications". 
     */
    @Test
    public void testSpacecraftStateToStateVector(){

        AbsoluteDate epoch = 
            new AbsoluteDate(1994, 5, 14, 13, 11, 20.59856, TimeScalesFactory.getUTC());
        Vector3D pos = new Vector3D(5036.736529 * 1e3, -10806.660797 * 1e3, -4534.633784 * 1e3);
        Vector3D vel = new Vector3D(2.6843855 * 1e3, -5.7595920 * 1e3, -2.4168093* 1e3);
        TimeStampedPVCoordinates timePosVel = 
            new TimeStampedPVCoordinates(epoch, new PVCoordinates(pos, vel));
        AbsolutePVCoordinates pv = 
            new AbsolutePVCoordinates(FramesFactory.getGCRF(), timePosVel);
        SpacecraftState scState = new SpacecraftState(pv);

        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        GeodeticPoint station = new GeodeticPoint(FastMath.toRadians(39.007),     // Geodetic latitude
                                                  FastMath.toRadians(-104.883),     // Longitude
                                         2194.56);                         // Altitude in [m]
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "Topocentric Horizon");

        TLE test = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");

        ObservedObject obj = new ObservedObject(0, null, null, topoHorizon, test);
        StateVector stateVec = obj.spacecraftStateToStateVector(scState);
        Vector3D posTopoHorizon = stateVec.getPositionVector();

        // Due to different angle definition, azimuth needs to get converted according to Vallado
        double actualAzOrekit = FastMath.toDegrees(posTopoHorizon.getAlpha());
        double actualAzVallado = 180 - (actualAzOrekit + 90.);

        // Compare range, azimuth and elevation with Vallado
        Assert.assertEquals(210.8777747, actualAzVallado, 1e-3);
        Assert.assertEquals(-5.9409535, FastMath.toDegrees(posTopoHorizon.getDelta()), 1e-3);
        Assert.assertEquals(11710812, posTopoHorizon.getNorm(), 1e2);
    }
}
