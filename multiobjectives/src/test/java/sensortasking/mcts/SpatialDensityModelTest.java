package sensortasking.mcts;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;

public class SpatialDensityModelTest {

    /** Test sensor according to Vallado Example 4-1. */
    Sensor sensor;

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
        density.createDensityModel(tleSeries, test.getDate(), referenceEpoch.shiftedBy(43200.));

    }
}
