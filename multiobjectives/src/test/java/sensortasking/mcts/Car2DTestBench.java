package sensortasking.mcts;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.util.FastMath;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.time.AbsoluteDate;

import benchtest.Car;

public class Car2DTestBench {
    @Before
    public void init() {
        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }

    @Test
    public void testBench(){
        /* Filter estimateCarA = new Filter();
        estimateCarA.run_ckf(init, P0.getData(), 5., 25.02);  */ 

        // Initialise first car 
        double[] initA = new double[]{0, -1, 5, 0};
        DiagonalMatrix P0A = new DiagonalMatrix(new double[]{0.1, 0.1, 0.01, 0.01});
        Car carA = new Car('A', initA, P0A.getData(), 0.);

        // Initialise second car
        double[] initB = new double[]{0, 1, 5, 0};
        DiagonalMatrix P0B = new DiagonalMatrix(new double[]{0.1, 0.1, 0.01, 0.01});
        Car carB = new Car('B', initB, P0B.getData(), 0.);

        List<ObservedObject> cars = new ArrayList<ObservedObject>();
        cars.add(carA);
        cars.add(carB);

        // Set up root node
        PropoagatedEnvironment env = new PropoagatedEnvironment(cars, new ArrayList<Integer>());
        Node root = new DecisionNode(1., 1, null, new AbsoluteDate(), env, 0, 0, 0.);

        // Set up fake sensor
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        Sensor sensor = new Sensor("Origin", fov, pos, 8., 7., 
                                    FastMath.toRadians(1.)/1., 7., FastMath.toRadians(5.));

        // Set up MCTS
        List<String> objectives = new ArrayList<String>(Arrays.asList( "TRACK_CAR"));
        MultiObjectiveMcts mcts = 
            new MultiObjectiveMcts(root, objectives, root.getEpoch(), 
                                   root.getEpoch().shiftedBy(15. * 60.), "Origin", cars, 
                                   null, sensor);
        mcts.run(2000, 2);
    }
}
