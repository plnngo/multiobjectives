package sensortasking.mcts;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
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
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AbsolutePVCoordinates;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public class TrackingObjectiveTest {

    /** Topocentric horizon frame.*/
    TopocentricFrame topoHorizon;

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

    @Test
    public void testSetMicroAction() throws IOException {

        


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
        
        AbsoluteDate sunset = new AbsoluteDate(2024, 4, 12, 0, 54, 0, TimeScalesFactory.getUTC());
        AbsoluteDate sunrise = new AbsoluteDate(2024, 4, 12, 11, 51, 0, TimeScalesFactory.getUTC());
        //AbsoluteDate date = new AbsoluteDate(2024, 4, 12, 2, 29, 30, TimeScalesFactory.getUTC());

    }
}
