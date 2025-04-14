package tools;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
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
import org.orekit.estimation.measurements.AngularRaDec;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class GaussianMixtureModelTest {
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
     * Bench test. Comparison of multivariate normal distribution using Matlab mvnpdf() function.
     */
    @Test
    public void testGetMultivariateGaussianLikelihood() {
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0);              // in [m]
        List<ObservedMeasurement<?>> meas = new ArrayList<ObservedMeasurement<?>>();
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topo = new TopocentricFrame(earth, pos, "Topocentric");
        GroundStation gs = new GroundStation(topo);
        AngularRaDec angles = new AngularRaDec(gs, FramesFactory.getEME2000(), new AbsoluteDate(),
                                               new double[]{1.5, -0.5}, new double[]{0., 0.}, 
                                               new double[]{1., 1.}, 
                                               new ObservableSatellite(0));
        meas.add(angles);
        RealVector ybar = new ArrayRealVector(new double[]{1., -1.});
        double[][] matrix = new double[2][2];
        matrix[0][0] = 0.9;
        matrix[0][1] = 0.4;
        matrix[1][0] = 0.4;
        matrix[1][1] = 0.3;
        RealMatrix pyy = new Array2DRowRealMatrix(matrix);
        double result = GaussianMixtureModel.getMultivariateGaussianLikelihood(meas, ybar, pyy);
        Assert.assertEquals(0.304591097939568, result, 1e-9);
    }
}
