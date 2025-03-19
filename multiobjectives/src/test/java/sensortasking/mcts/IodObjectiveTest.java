package sensortasking.mcts;

import java.io.File;
import org.hipparchus.util.FastMath;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;

import tools.GaussianMixtureModel;

public class IodObjectiveTest {

    @Test
     public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }

    @Test
    public void test_optical_car_gmm() {
       
        // Measurement vector
        double ra = FastMath.toRadians(10.);
        double dec = FastMath.toRadians(-2.);
        double dra = FastMath.toRadians(15.)/3600.;
        double ddec = FastMath.toRadians(3.)/3600.;
        double[] tracklet = new double[]{ra, dec, dra, ddec};

        // CAR limits
        double a_max = 50000.*1000.;            // m
        double a_min = 0.;            // m
        double e_max = 0.4;

        // Range interval
        double[] rho_vect = new double[10000];
        int count = 0;
        for (int i=0; i<rho_vect.length; i++) {
            rho_vect[i] = count;
            count = count + 5000;
        }

        // Desired maximum standard deviation in range 
        double sigma_rho_desired = 500.;       // m
        double sigma_drho_desired = 80.;       // m/s

        // Measurement noise
        double arcsec2rad = 1./3600. * FastMath.PI/180.;
        double[] meas_noise = new double[]{0.4 * arcsec2rad,
                                           0.4 * arcsec2rad,
                                           0.07 * arcsec2rad,
                                           0.07 * arcsec2rad};

        IodObjective obj = new IodObjective(tracklet, null, a_max, a_min, e_max, meas_noise);
        //obj.car_drho_limits(null, rho_vect, true);
        GaussianMixtureModel gmm = obj.optical_car_gmm(null, rho_vect, sigma_rho_desired, sigma_drho_desired);
        obj.car_gmm_to_eci(gmm, meas_noise);

    }
}
