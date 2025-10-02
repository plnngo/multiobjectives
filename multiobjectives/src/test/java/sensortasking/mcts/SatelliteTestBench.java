package sensortasking.mcts;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
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
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import benchtest.RewardFunction;

public class SatelliteTestBench {
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
    public void testBenchTracking(){

        // Date
        AbsoluteDate date = new AbsoluteDate(2025, 3, 24, 22, 1, 2.62, TimeScalesFactory.getUTC());

       // Set up fake sensor
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(0.25), FastMath.toRadians(0.25));
        Sensor sensor = new Sensor("Origin", fov, pos, 8., 7., 
                                    FastMath.toRadians(1.)/1., 7., FastMath.toRadians(5.));
        
        // Set up general frames
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame j2000 = FramesFactory.getEME2000();

        // Set up topocentric sencor frame
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");
        Transform horizonToEci = topohorizon.getTransformTo(j2000, date);  // date has to be the measurement epoch
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
        Transform eciToTopo = new Transform(date, coordinatesStationEci.negate());
        Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);

        // Set up initial pointning
        AngularDirection initPointing = 
            new AngularDirection(topocentric, new double[]{0.,0.}, AngleType.RADEC, 1.);

        // Set reward function
        RewardFunction reward = RewardFunction.IMMEDIATE_REWARD;

        // Set up targets
        List<ObservedObject> ooi = ESDConferenceTrackingTask.generateListOfCandidates(date);
        
        // Set up root node
        PropoagatedEnvironment env = new PropoagatedEnvironment(ooi, new ArrayList<Integer>());
        Node root = new DecisionNode(1., 1, initPointing, date, env, 0, 0, 0.);

        // Set up MCTS
        List<String> objectives = new ArrayList<String>(Arrays.asList( "TRACK"));
        MultiObjectiveMcts mcts = 
            new MultiObjectiveMcts(root, objectives, root.getEpoch(), 
                                   root.getEpoch().shiftedBy(1. * 60.), "Origin", ooi, 
                                   null, sensor, reward, true);
        List<Node> strategy = mcts.run(10);

    }
    
}
