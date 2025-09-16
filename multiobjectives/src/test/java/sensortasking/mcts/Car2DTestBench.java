package sensortasking.mcts;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

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
import benchtest.CarTrackingObjective;
import benchtest.Filter;
import benchtest.RewardFunction;

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
    public void testBench() throws IOException{
        long start = System.currentTimeMillis();

        // Initialise first car 
        double[] initA = new double[]{0, -1, 5, 0};
        //double[] initA = new double[]{-2, 0, 0., -FastMath.PI/90.};
        DiagonalMatrix P0A = new DiagonalMatrix(new double[]{0.5, 0.5, 0.05, 0.05});
        //DiagonalMatrix P0A = new DiagonalMatrix(new double[]{0.01, 0.01, 0.001, 0.001});

        Car carA = new Car('A', initA, P0A.getData(), 0.);

        // Initialise second car
        double[] initB = new double[]{0, 1, 5, 0};
        //double[] initB = new double[]{2, 0, 0., FastMath.PI/90.};
        DiagonalMatrix P0B = new DiagonalMatrix(new double[]{0.01, 0.01, 0.001, 0.001});
        //DiagonalMatrix P0B = new DiagonalMatrix(new double[]{0.5, 0.5, 0.05, 0.05});

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
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(0.25), FastMath.toRadians(0.25));
        Sensor sensor = new Sensor("Origin", fov, pos, 8., 7., 
                                    FastMath.toRadians(1.)/1., 7., FastMath.toRadians(5.));

        // Set reward function
        RewardFunction reward = RewardFunction.IMMEDIATE_REWARD;

        // Set up MCTS
        List<String> objectives = new ArrayList<String>(Arrays.asList( "TRACK_CAR"));
        MultiObjectiveMcts mcts = 
            new MultiObjectiveMcts(root, objectives, root.getEpoch(), 
                                   root.getEpoch().shiftedBy(20. * 60.), "Origin", cars, 
                                   null, sensor, reward);
        List<Node> strategy = mcts.run(4500);

        evaluation(strategy);
        long finish = System.currentTimeMillis();
        long timeElapsed = finish - start;
        System.out.println("Run time in milliseconds: " + timeElapsed);
    }

    private void evaluation(List<Node> strategy) throws IOException {
        try (FileWriter writer = new FileWriter("Strategy_Car_Option21_AlargeCov_discount1_range_20min.csv")) {
            writer.append("car,time,meas,x1,x2,x3,x4,std1,std2,std3,std4\n");
            char id = 'o';
            double noisyAngle = Double.MIN_VALUE;
            for (Node current : strategy) {
                if (current.getClass().getSimpleName().equals("ChanceNode")) {
                    id = ((CarTrackingObjective)((ChanceNode) current).getMacro()).getLastUpdated();
                    AngularDirection task = ((ChanceNode) current).getMicro();
                    Random r = new java.util.Random();
                    noisyAngle = /* r.nextGaussian() * FastMath.sqrt(Filter.Rk) + */  task.getScale();
                } else {
                    List<ObservedObject> targets = 
                        ((DecisionNode)current).getEnvironment().getStateTracking();
                    for (ObservedObject target : targets) {
                        Car car = (Car)target;
                        if (car.getIdentifier() == id) {
                            double[] state = car.getStateArray();
                            double[][] cov = car.getCov();
                            // standard deviation
                            double[] std = new double[cov.length];
                            for (int i=0; i<cov.length; i++) {
                                std[i] = FastMath.sqrt(cov[i][i]);
                            }

                            writer.append(id);
                            writer.append(",");
                            writer.append(Double.toString(car.getTime()));
                            writer.append(",");
                            writer.append(Double.toString(noisyAngle));
                            writer.append(",");


                            // Write state vector
                            for (int j = 0; j < state.length; j++) {
                                writer.append(Double.toString(state[j]));
                                writer.append(",");
                            }

                            // Write std vector
                            for (int j = 0; j < std.length; j++) {
                                writer.append(Double.toString(std[j]));
                                if (j < std.length - 1) {
                                    writer.append(",");
                                }
                            }
                            writer.append("\n");
                        }
                    }
                }
            }
        }
    }
}
