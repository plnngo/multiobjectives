package sensortasking.mcts;

import java.io.File;
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
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.tle.TLE;
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

    @Before
    public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        /* // Location at United States Air Force Academy according to Vallado Example 4-1
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),   // Geodetic latitude
                                              FastMath.toRadians(-104.883),   // Longitude
                                              2194.56);              // in [m]

        // Set up topocentric horizon frame
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               earthFrame);
        this.topoHorizon = new TopocentricFrame(earth, pos, "United States Air Force Academy"); */
    }

    @Test
    public void testSetMicroAction() throws IOException {

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
        AbsolutePVCoordinates pv = 
            new AbsolutePVCoordinates(eci, new TimeStampedPVCoordinates(date, pvTeme));
        SpacecraftState scStateTeme = new SpacecraftState(pv);
        StateVector stateTeme = ObservedObject.spacecraftStateToStateVector(scStateTeme, teme);

        Transform eciToEcef = eci.getTransformTo(ecef, date);
        PVCoordinates pvEcef =  eciToEcef.transformPVCoordinates(pvEci);

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
        StateCovariance covTeme = new StateCovariance(covMatrix, date, teme, OrbitType.CARTESIAN, PositionAngle.MEAN);
        CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        CartesianCovariance stateCovTeme = 
            ObservedObject.stateCovToCartesianCov(orbit, covTeme, teme);

        // Generate list of object of interest OOI
        TLE testTle = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");
        TLE.stateToTLE(scStateTeme, testTle);
        ObservedObject test = new ObservedObject(123, stateTeme , stateCovTeme, teme, testTle);
        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(test);

        // Retrieve coordinates of ground station
        double lat = pvEcef.getPosition().getAlpha();
        double lon = pvEcef.getPosition().getDelta();
        System.out.println("Lat " + FastMath.toDegrees(lat));
        System.out.println("Lon " + FastMath.toDegrees(lon));

        GeodeticPoint pos = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]

        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, pos, "Generic Station");

        // Call test function 
        TrackingObjective trackTask = new TrackingObjective(ooi, topoHorizon);
        trackTask.setMicroAction(date.shiftedBy(10));



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
