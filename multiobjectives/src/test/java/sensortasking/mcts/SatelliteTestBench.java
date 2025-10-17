package sensortasking.mcts;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.events.Action;
import org.hipparchus.util.FastMath;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import benchtest.RewardFunction;
import benchtest.Satellite;
import data.TleLoader;

public class SatelliteTestBench {

    Sensor sensor;

    TopocentricFrame topohorizon;

    AbsoluteDate startSim;

    AbsoluteDate endSim;

    @Before
    public void init() {
        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Set up fake sensor
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(0.25), FastMath.toRadians(0.25));
        sensor = new Sensor("Origin", fov, pos, 8., 7., 
                                 FastMath.toRadians(1.)/1., 7., FastMath.toRadians(5.));
        
        // Set up general frames
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        

        // Set up topocentric sencor frame
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");

        startSim = new AbsoluteDate(2025, 3, 24, 22, 1, 2.62, TimeScalesFactory.getUTC());

        endSim = startSim.shiftedBy(20. * 60.);
    }

    @Test
    public void testBenchTracking() throws IOException{

        long start = System.currentTimeMillis();      

        Frame j2000 = FramesFactory.getEME2000();
        Transform horizonToEci = topohorizon.getTransformTo(j2000, startSim);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(startSim, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        // Set up initial pointning
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC, 1.);

        // Set reward function
        RewardFunction reward = RewardFunction.IMMEDIATE_REWARD;

        // Set up targets
        List<ObservedObject> ooi = ESDConferenceTrackingTask.generateListOfCandidates(startSim);
        
        // Set up root node
        PropoagatedEnvironment env = new PropoagatedEnvironment(ooi, new ArrayList<Integer>());
        Node root = new DecisionNode(1., 1, initPointing, startSim, env, 0, 0, 0.);

        // Set up MCTS
        List<String> objectives = new ArrayList<String>(Arrays.asList( "TRACK"));
        MultiObjectiveMcts mcts = 
            new MultiObjectiveMcts(root, objectives, root.getEpoch(), 
                                   endSim, "Origin", ooi, 
                                   null, sensor, reward, true);
        //List<Node> strategy = mcts.run(1240);
        List<Node> strategy = mcts.run(100);
        long finish = System.currentTimeMillis();
        long timeElapsed = finish - start;
        System.out.println("Run time in milliseconds: " + timeElapsed);
        evaluation(strategy);

    }

    private void evaluation(List<Node> strategy) throws IOException {
        try (FileWriter writer = new FileWriter("Strategy_Orbit_Option21_samCov_discount0_rangeBearing_20min.csv")) {
            writer.append("satellite,time,ra, dec, range,x1,x2,x3,x4,x5, x6, std1,std2,std3,std4, std5, std6\n");
            long id = 0;
            double ra = Double.MIN_VALUE;
            double dec = Double.MIN_VALUE;
            double range = Double.MIN_VALUE;
            AbsoluteDate startCampaign = new AbsoluteDate();
            for (Node current : strategy) {
                if (current.getClass().getSimpleName().equals("ChanceNode")) {
                    id = ((TrackingObjective)((ChanceNode) current).getMacro()).getLastUpdated();
                    AngularDirection task = ((ChanceNode) current).getMicro();
                    Random r = new java.util.Random();
                    ra = task.getAngle1();
                    dec = task.getAngle2();
                    range = /* r.nextGaussian() * FastMath.sqrt(Filter.Rk) + */  task.getScale();
                } else {
                    if(current.getId() == 0) {
                        // root node
                        startCampaign = current.getEpoch();
                    }
                    List<ObservedObject> targets = 
                        ((DecisionNode)current).getEnvironment().getStateTracking();
                    for (ObservedObject target : targets) {
                        Satellite sat = (Satellite)target;
                        if (sat.getId() == id) {
                            double[] statePos = sat.getState().getPositionVector().toArray();
                            double[] stateVel = sat.getState().getVelocityVector().toArray();

                            double[][] cov = sat.getCovariance().getCovarianceMatrix().getData();
                            // standard deviation
                            double[] std = new double[cov.length];
                            for (int i=0; i<cov.length; i++) {
                                std[i] = FastMath.sqrt(cov[i][i]);
                            }

                            // Compute time
                            double time = sat.getEpoch().durationFrom(startCampaign);

                            writer.append(Long.toString(id));
                            writer.append(",");
                            writer.append(Double.toString(time));
                            writer.append(",");
                            writer.append(Double.toString(ra));
                            writer.append(",");
                            writer.append(Double.toString(dec));
                            writer.append(",");
                            writer.append(Double.toString(range));
                            writer.append(",");


                            // Write state vector
                            for (int j = 0; j < statePos.length; j++) {
                                writer.append(Double.toString(statePos[j]));
                                writer.append(",");
                            }
                            for (int j = 0; j < stateVel.length; j++) {
                                writer.append(Double.toString(stateVel[j]));
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
    
    @Test
    public void testBenchSearching() throws IOException {

        // Parse spacetrack entries into list of TLEs
        File tleFile = new File( System.getProperty("user.dir") 
                                    + "\\src\\main\\java\\data\\Catalogue_16_10_2025.txt");
        List<TLE> tles = TleLoader.parse(tleFile);

        // Set up event logger
        EventsLogger logger = new EventsLogger();

        // Set up propagators
        final List<Propagator> propagators = new ArrayList<>();
        for (TLE entry : tles) {
            try {
                TLEPropagator prop = TLEPropagator.selectExtrapolator(entry);

                // Set field of regard detector as event detector
                double maxcheck  = 60.0;
                double threshold =  0.001;
                double elevation = FastMath.toRadians(5.);
                EventDetector forVisibility =
                    new ElevationDetector(maxcheck, threshold, topohorizon).
                    withConstantElevation(elevation).
                    withHandler((s, detector, increasing) -> {
                                        System.out.println(" Visibility on " +
                                                        entry.getSatelliteNumber() +
                                                        (increasing ? " begins at " : " ends at ") +
                                                        s.getDate());
                                        return increasing ? Action.CONTINUE : Action.STOP;
                                    });
                prop.addEventDetector(logger.monitorDetector(forVisibility));
                propagators.add(prop);
            } catch (OrekitException e) {
                System.out.println(e.getMessage() + " skip object " + entry.getSatelliteNumber());
                continue;
            }  
        }
        int counter = 0;
        for (Propagator prop : propagators) {
            try {
                prop.propagate(startSim, endSim);
                counter++;
            } catch (OrekitException e) {
                System.out.println("Propagation failed for one object: " + e.getMessage());
            }
        }
        System.out.println("Number of TLEs: " + tles.size());

        System.out.println("counter: " + counter);

        System.out.println(logger.getLoggedEvents().size());
    }
}
