package sensortasking.mcts;

import java.io.File;
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
import org.orekit.time.TimeScale;
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
    public void testCreateDensityModel(){

        TLE test = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");
        List<TLE> tleSeries = new ArrayList<TLE>();
        tleSeries.add(test);

        AbsoluteDate referenceEpoch = new AbsoluteDate(test.getDate(), 86400.);

        SpatialDensityModel density = new SpatialDensityModel(sensor, referenceEpoch);
        //density.createDensityModel(tleSeries, test.getDate(), referenceEpoch.shiftedBy(43200.));

    }

    @Test
    public void testZeroOutRegionMoonInDensityModel(){
        
        AbsoluteDate start = new AbsoluteDate(2024, 4, 24, 1, 7, 0, TimeScalesFactory.getUTC());
        AbsoluteDate end = new AbsoluteDate(2024, 4, 24, 11, 19, 0, TimeScalesFactory.getUTC());
        AbsoluteDate date = new AbsoluteDate(start, 5*60*60);   // reference date: 5 hours after start date
        SpatialDensityModel densityModel = new SpatialDensityModel(sensor, date);
        densityModel.zeroOutRegionMoonInDensityModel(start, end);
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
