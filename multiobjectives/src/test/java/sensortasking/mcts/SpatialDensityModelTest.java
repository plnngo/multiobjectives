package sensortasking.mcts;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class SpatialDensityModelTest {

    /** Test sensor according to Vallado Example 4-1. */
    Sensor sensor;

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

        // Settings in terms of duration according to Frueh
        double exposureT = 8.;      //in [s]
        double readoutT = 7.;       //in [s]
        double slewT = 9.;          //in [s]

        // Settings of sensor     
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(0.59), FastMath.toRadians(0.73));
        double slewVel = fov.getHeight()/slewT;
        double cutOff = FastMath.toRadians(5.);

        // Location at United States Air Force Academy according to Vallado Example 4-1
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),   // Geodetic latitude
                                              FastMath.toRadians(-104.883),   // Longitude
                                              2194.56);              // in [m]

        sensor = new Sensor("United States Air Force Academy", fov, pos, exposureT, readoutT,
                                 slewVel, cutOff);

        // Set up topocentric horizon frame
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               earthFrame);
        this.topoHorizon = new TopocentricFrame(earth, sensor.getPosition(), "station");
    }

    @Test
    public void testDimensionsDensityModel() {
        TLE test = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");
        List<TLE> tleSeries = new ArrayList<TLE>();
        tleSeries.add(test);

        AbsoluteDate referenceEpoch = new AbsoluteDate(test.getDate(), 86400.);

        SpatialDensityModel density = new SpatialDensityModel(sensor, referenceEpoch);
        int[][] model = density.getDensityModel();

        // Assign expected dimensions
        int expectedNumElPatches = 145;     // number of rows in density model
        int expectedNumAzPatches = 494;     // number of columns in density model
        Assert.assertEquals(expectedNumElPatches, model.length);
        Assert.assertEquals(expectedNumAzPatches, model[0].length);
    }
    
    @Test
    public void testCreateDensityModel() throws IOException{

/*         TLE test = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");
        List<TLE> tleSeries = new ArrayList<TLE>();
        tleSeries.add(test);

        AbsoluteDate referenceEpoch = new AbsoluteDate(test.getDate(), 86400.);

        SpatialDensityModel density = new SpatialDensityModel(sensor, referenceEpoch); */
        //density.createDensityModel(tleSeries, test.getDate(), referenceEpoch.shiftedBy(43200.));

/*         AbsoluteDate start = new AbsoluteDate(2024, 4, 24, 1, 4, 0, TimeScalesFactory.getUTC());
        AbsoluteDate end = new AbsoluteDate(2024, 4, 24, 11, 35, 0, TimeScalesFactory.getUTC());
        AbsoluteDate date = new AbsoluteDate(start, 5*60*60);   // reference date: 5 hours after start date

        Orbit perfectGeo = new KeplerianOrbit(7000*1e3, 0., FastMath.toRadians(50.), 0., 0., 5.39, PositionAngle.MEAN, FramesFactory.getGCRF(), start, Constants.WGS84_EARTH_MU);
        Propagator kepProp = new KeplerianPropagator(perfectGeo);
        //compute expected result
        ElevationDetector elevationDetector =
                new ElevationDetector(this.topoHorizon)
                        .withConstantElevation(this.sensor.getElevCutOff())
                        .withMaxCheck(5.0);
        EventsLogger logger = new EventsLogger();
        kepProp.addEventDetector(logger.monitorDetector(elevationDetector));
        kepProp.propagate(end);
        List<LoggedEvent> expected = logger.getLoggedEvents();

        SpacecraftState state1 = kepProp.propagate(new AbsoluteDate(2024, 4, 24, 1, 48, 0, TimeScalesFactory.getUTC()));
        SpacecraftState state2 = kepProp.propagate(new AbsoluteDate(2024, 4, 24, 1, 50, 0, TimeScalesFactory.getUTC()));
        SpacecraftState state3 = kepProp.propagate(new AbsoluteDate(2024, 4, 24, 1, 52, 0, TimeScalesFactory.getUTC()));

 */

        //TimeStampedPVCoordinates timePvGcrf = new TimeStampedPVCoordinates(date.shiftedBy(-60.), expected.get(0).getState().getPVCoordinates());

        /* PVCoordinates pvTopoHorizon = expected.get(0).getState().getFrame()
                                        .getTransformTo(this.topoHorizon, expected.get(0).getState().getDate())
                                        .transformPVCoordinates(expected.get(0).getState().getPVCoordinates());
        TimeStampedPVCoordinates timePvTopoHorizon = new TimeStampedPVCoordinates(date.shiftedBy(-60.), pvTopoHorizon); */
/*         Orbit newObject = new CartesianOrbit(timePvGcrf, FramesFactory.getGCRF(), Constants.WGS84_EARTH_MU);

        Propagator newProp = new KeplerianPropagator(newObject);
        SpacecraftState state = newProp.propagate(date); */
/* 
        PVCoordinates pvTopoHorizon = state3.getFrame()
                                        .getTransformTo(this.topoHorizon, date)
                                        .transformPVCoordinates(state3.getPVCoordinates());

        System.out.println(pvTopoHorizon.getPosition().getDelta());  */

        //perfectGeo.getPVCoordinates();


        // Load fake TLEs
/*         TLE testTle = new TLE("1 25338U 98030A   24102.60546611  .00000200  00000-0  10064-3 0  9995 ", 
                              "2 25338  98.5757 131.1050 0010951 153.9296 206.2438 14.26531185347696"); */
        String workingDir = System.getProperty("user.dir");
        String fakeTleDataDir = "\\src\\test\\java\\resources\\test-data\\noaa15fakeTles.tle";
        File testData = new File(workingDir + fakeTleDataDir);
        BufferedReader reader = new BufferedReader(new FileReader(testData));
        List<TLE> tleSeries = new ArrayList<TLE>();
        String current = "";
        while((current = reader.readLine())!=null){
            tleSeries.add(new TLE(current, reader.readLine()));
        }
        reader.close();
        
        AbsoluteDate sunset = new AbsoluteDate(2024, 4, 12, 0, 54, 0, TimeScalesFactory.getUTC());
        AbsoluteDate sunrise = new AbsoluteDate(2024, 4, 12, 11, 51, 0, TimeScalesFactory.getUTC());
        AbsoluteDate date = new AbsoluteDate(2024, 4, 12, 2, 29, 30, TimeScalesFactory.getUTC());
        SpatialDensityModel model = new SpatialDensityModel(sensor, date);
        int[][] spatialDensity = model.createDensityModel(tleSeries, sunset, sunrise, false);

        // Expected patches to contain space objects
        List<int[]>expectedPatches = new ArrayList<int[]>();
        expectedPatches.add(new int[]{4, 63});
        expectedPatches.add(new int[]{7, 163});
        expectedPatches.add(new int[]{9, 159});
        expectedPatches.add(new int[]{12, 95});
        expectedPatches.add(new int[]{29, 137});
        expectedPatches.add(new int[]{32, 138});
        expectedPatches.add(new int[]{33, 138});
        expectedPatches.add(new int[]{61, 261});
        expectedPatches.add(new int[]{63, 261});

        // Compare
        for (int[] expectedPatch : expectedPatches){
            int az_col = expectedPatch[1];
            int el_row = expectedPatch[0];
            if (az_col==159 && el_row==9) {
                Assert.assertEquals(2, spatialDensity[el_row][az_col]);
            } else {
                Assert.assertEquals(1, spatialDensity[el_row][az_col]);
            }
        }

        model = new SpatialDensityModel(sensor, date);
        int[][] spatialDensityZeroedUnfavourCondition = 
            model.createDensityModel(tleSeries, sunset, sunrise, true);
        model.writeDataLineByLine("TestFakeSatellitesZeroed.csv");

        // Compare
        for (int[] expectedPatch : expectedPatches){
            int az_col = expectedPatch[1];
            int el_row = expectedPatch[0];
            if ((az_col==63 && el_row==4) || (az_col==95 && el_row==12)) {
                Assert.assertEquals(1, spatialDensityZeroedUnfavourCondition[el_row][az_col]);
            } else {
                Assert.assertEquals(-1, spatialDensityZeroedUnfavourCondition[el_row][az_col]);
            }
        }

        //model.writeDataLineByLine("TestFakeSatellites.csv");

       /*  TLEPropagator propagator = TLEPropagator.selectExtrapolator(tleSeries.get(0));
        //AbsoluteDate date = new AbsoluteDate(2024, 4, 12, 2, 29, 35, TimeScalesFactory.getUTC());
        
        SpacecraftState finalState = propagator.propagate(date);       // state in TEME
        PVCoordinates pvTeme = finalState.getPVCoordinates();
        PVCoordinates pvTopoHorizon = finalState.getFrame()
                                                .getTransformTo(this.topoHorizon, date)
                                                .transformPVCoordinates(pvTeme);

        // Extract angular position of space object 
        double azimuth = pvTopoHorizon.getPosition().getAlpha();        // between -Pi and +Pi
        double elevation = pvTopoHorizon.getPosition().getDelta();      // between -Pi/2 and +Pi/2
        System.out.println("Azimuth " + FastMath.toDegrees(0.5*FastMath.PI - azimuth));
        System.out.println("Elevation " + FastMath.toDegrees(elevation)); */


    }

    @Test
    public void testZeroOutRegionMoonInDensityModel(){
        //Day of full moon
        AbsoluteDate start = new AbsoluteDate(2024, 4, 24, 1, 4, 0, TimeScalesFactory.getUTC());
        AbsoluteDate end = new AbsoluteDate(2024, 4, 24, 11, 35, 0, TimeScalesFactory.getUTC());
/*         AbsoluteDate start = new AbsoluteDate(2024, 4, 15, 17, 11, 0, TimeScalesFactory.getUTC());
        AbsoluteDate end = new AbsoluteDate(2024, 4, 16, 8, 28, 0, TimeScalesFactory.getUTC()); */
        AbsoluteDate date = new AbsoluteDate(start, 5*60*60);   // reference date: 5 hours after start date
        SpatialDensityModel densityModel = new SpatialDensityModel(sensor, date);
        int[][] moonZeroedOut = densityModel.zeroOutRegionMoonInDensityModel(start, end);

        // Check if following patches are zeroed out
        int row = 0;
        for (int col=271; col<=423; col++) {
            Assert.assertEquals(-1, moonZeroedOut[row][col]);
        }
        int col = 271;
        for (row=0; row<=21; row++) {
            Assert.assertEquals(-1, moonZeroedOut[row][col]);
        }
        Assert.assertEquals(-1, moonZeroedOut[21][273]);
        Assert.assertEquals(-1, moonZeroedOut[20][280]);
        Assert.assertEquals(-1, moonZeroedOut[12][339]);
        Assert.assertEquals(-1, moonZeroedOut[1][416]);

        // Check if following patches are not zeroed out
        Assert.assertEquals(0, moonZeroedOut[2][410]);
        Assert.assertEquals(0, moonZeroedOut[8][368]);
        Assert.assertEquals(0, moonZeroedOut[22][271]);
        //densityModel.writeDataLineByLine("TestDensityModel.csv");
    }

    @Test
    public void testAngularDirectionToGridPosition(){

        double az = FastMath.toRadians(50.);
        double elev = FastMath.toRadians(75.);
        AngularDirection objectTopoHorizon = 
            new AngularDirection(this.topoHorizon, new double[]{az, elev}, AngleType.AZEL);
        SpatialDensityModel density = new SpatialDensityModel(sensor, new AbsoluteDate());
        
        int[] actual = density.angularDirectionToGridPosition(objectTopoHorizon);
        Assert.assertArrayEquals(new int[]{119, 68}, actual);
    }
}
