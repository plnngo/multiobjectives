package sensortasking.stripescanning;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
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
import org.orekit.estimation.iod.IodGooding;
import org.orekit.estimation.measurements.AngularRaDec;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import sensortasking.mcts.AngleType;
import sensortasking.mcts.AngularDirection;
import sensortasking.mcts.Fov;
import sensortasking.mcts.Sensor;

public class Simulation {

    List<TLE> tleSeries = new ArrayList<TLE>();

    @Before
    public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        readTles();

        //propagateTles();
    }


    @Test
    public void propagateTles() {

        // Set up
        double stepT = 10.;
        AbsoluteDate startCampaign = 
            //new AbsoluteDate(2025, 3, 24, 20, 58, 0, TimeScalesFactory.getUTC());
            new AbsoluteDate(2025, 3, 24, 21, 58, 0, TimeScalesFactory.getUTC());

        AbsoluteDate endCampaign = startCampaign.shiftedBy(60.*12);
        
        // Prepare output file
        String filename = endCampaign.toString().replaceAll("[^a-zA-Z0-9]", "") + ".csv";
        
        try (FileWriter writer = new FileWriter(filename)) {
            // Write header
            writer.append("UTC,NORAD,x [J2000  - km],y [J2000 - km],z [J2000 - km],vx [J2000 - km/s],vy [J2000 - km/s],vz [J2000 - km/s],alpha [rad],delta [rad]\n");
        
            // Recover short term perturbations using SGP4
            List<SpacecraftState> osculating = new ArrayList<SpacecraftState>();
            for (TLE tle :  this.tleSeries) {
                TLEPropagator sgp4 = TLEPropagator.selectExtrapolator(tle); 
                SpacecraftState state = sgp4.getInitialState();
                osculating.add(state);

                // Use Kep propagator for further dynamics modelling
                
                KeplerianPropagator kepPropo = new KeplerianPropagator(state.getOrbit());
                for (AbsoluteDate extrapDate = startCampaign;
                    extrapDate.compareTo(endCampaign) <= 0;
                    extrapDate = extrapDate.shiftedBy(stepT)) {
                    SpacecraftState propState = kepPropo.propagate(extrapDate);
                    writer.append(extrapDate.toString() + ",");
                    writer.append(String.valueOf(tle.getSatelliteNumber()) + ",");
                    writer.append(String.valueOf(propState.getPosition().getX() * 1e3) + ",");
                    writer.append(String.valueOf(propState.getPosition().getY() * 1e3) + ",");
                    writer.append(String.valueOf(propState.getPosition().getZ() * 1e3) + ",");
                    writer.append(String.valueOf(propState.getPVCoordinates().getVelocity().getX() * 1e3) + ",");
                    writer.append(String.valueOf(propState.getPVCoordinates().getVelocity().getY() * 1e3) + ",");
                    writer.append(String.valueOf(propState.getPVCoordinates().getVelocity().getZ() * 1e3) + ",");  
                    writer.append(String.valueOf(propState.getPosition().getAlpha()) + ",");
                    writer.append(String.valueOf(propState.getPosition().getDelta()) + "\n");
                }
            }
            System.out.println("CSV file saved successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
        
        
    public void readTles() {
        String workingDir = System.getProperty("user.dir");
        String fakeTleDataDir = "\\src\\test\\java\\resources\\test-data\\GEO_22_03_2025.3le";
        File testData = new File(workingDir + fakeTleDataDir);
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(testData));
            
            String current = "";
            while((current = reader.readLine())!=null){
                //System.out.println(current);
                if(current.charAt(0) != '0') {
                    try{
                        tleSeries.add(new TLE(current, reader.readLine()));
                    } catch (NumberFormatException e) {
                        // TODO Auto-generated catch block
                        continue;
                    }
                }
            }
            reader.close();

        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Test
    public void registerStripeDetections() {

        // Constants
        double geoDistance = Constants.WGS84_EARTH_EQUATORIAL_RADIUS + 35786 * 1e3;  // in m
        final String COMMA_DELIMITER = ",";

        // Dates
        AbsoluteDate startCampaign = new AbsoluteDate(2025, 3, 24, 21, 58, 0, TimeScalesFactory.getUTC());
        AbsoluteDate endCampaign = startCampaign.shiftedBy(60. * 60.);

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),      // Geodetic latitude
                                              FastMath.toRadians(-37.),      // Longitude
                                     0.);                           // in [m]

        // Settings of sensor in terms of duration according to Frueh
        double exposureT = 8.;      //in [s]
        double readoutT = 7.;       //in [s]
        double settlingT = readoutT;//in [s]
        double cutOff = FastMath.toRadians(5.);

        // Settings of sensor     
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposureT, readoutT, slewVel, settlingT, cutOff);
        int numExpose = 5;
        AngularDirection initialPointing = new AngularDirection(FramesFactory.getEME2000(), 
                                                                new double[]{0., 0.}, 
                                                                AngleType.RADEC, geoDistance);
        initialPointing.setDate(startCampaign);

        Tasking survey = new Tasking(sensor,startCampaign, endCampaign, numExpose);
        Stripe[] stripes = survey.computeScanStripes();
        Stripe stripe = stripes[1];

        // Sensor schedule
        List<AngularDirection> schedule =
            callStripeScanGeoFrame(sensor, stripe, numExpose, startCampaign, initialPointing);

        // Extract angular positions of space objects
        String workingDir = System.getProperty("user.dir");
        String nameFile = "\\20250324T221000000Z.csv";
        List<List<String>> records = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(new FileReader(workingDir + nameFile))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] values = line.split(COMMA_DELIMITER);
                records.add(Arrays.asList(values));
            }
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        List<AngularDirection> candidates = parseToAngularDirection(records);
        List<List<AngularDirection>> registered = new ArrayList<List<AngularDirection>>();

        for(int decIndex=0; decIndex<schedule.size(); decIndex++) {
            List<AngularDirection> registeredDecField = new ArrayList<AngularDirection>();

            for(AngularDirection obj : candidates) {

                AngularDirection decField = schedule.get(decIndex);
                
                double[] raRange = new double[]{decField.getAngle1() - fov.getWidth()/2, 
                                                decField.getAngle1() + fov.getWidth()/2};
                double[] decRange = new double[]{decField.getAngle2() - fov.getHeight()/2,
                                                decField.getAngle2() + fov.getHeight()/2};
                if(checkInAngularRange(obj, raRange, decRange) 
                    && checkInTimeRange(obj, 
                                        decField.getDate().shiftedBy(-sensor.getExposureT()/2), 
                                        decField.getDate().shiftedBy(sensor.getExposureT()/2))) {
                        boolean alreadyRegistered = false;
                    for(AngularDirection dir : registeredDecField) {
                        if(dir.getName().equals(obj.getName())) {
                            alreadyRegistered = true;
                            break;
                        }
                    }
                    if (alreadyRegistered == false) {
                        registeredDecField.add(obj);
                        System.out.println(obj.getName());
                        System.out.println("Position of object " + FastMath.toDegrees(obj.getAngle1()) + " and " + FastMath.toDegrees(obj.getAngle2()));
                        System.out.println("Exposer shoot: " + decIndex);
                        //System.out.println("Position of declination field " + FastMath.toDegrees(decField.getAngle1()) + " and " + FastMath.toDegrees(decField.getAngle2()));
                        System.out.println(obj.getDate().toString());
                    }
                }
            }
            registered.add(registeredDecField);
        }
        
        for(int i=0; i<registered.size(); i++) {
            System.out.println(i + ". dec field registered " + registered.get(i).size() + " objects.");
            for (int j=0; j<registered.get(i).size(); j++) {
                System.out.print(registered.get(i).get(j).getName() + " - ");
                if (j == registered.get(i).size() -1) {
                    System.out.println();
                }
            }
        }
    }
    private static boolean checkInTimeRange(AngularDirection obj, AbsoluteDate start, AbsoluteDate end) {
        if(obj.getDate().compareTo(start) >= 0 && obj.getDate().compareTo(end) <= 0){
            return true;
        }
        return false;
    }

    private static boolean checkInAngularRange(AngularDirection obj, double[] raRange, double[] decRange) {
        double ra = obj.getAngle1() ;
        if(ra< 0.) {
            ra += 2*FastMath.PI;
        }

        if(raRange[0] < ra && ra < raRange[1] 
                && decRange[0] < obj.getAngle2() && obj.getAngle2() < decRange[1]) {
                    return true;
            }
        return false;
    }

    private static List<AngularDirection> parseToAngularDirection(List<List<String>> records) {

        // Parse into angular directions
        List<AngularDirection> geo = new ArrayList<AngularDirection>();
        for (int i=1; i<records.size(); i++) {

            // Parse row by row
            List<String> row = records.get(i);

            // // Get date
            AbsoluteDate date = new AbsoluteDate(row.get(0).replace(" ", "T"), TimeScalesFactory.getUTC());
            String noradId = row.get(1);
            Vector3D pos = new Vector3D(Double.parseDouble(row.get(2)), 
                                        Double.parseDouble(row.get(3)), 
                                        Double.parseDouble(row.get(4)));
            double distance = pos.getNorm();

            double[] angles = new double[]{pos.getAlpha(), pos.getDelta()};
                       
            AngularDirection angularPos = 
                new AngularDirection(FramesFactory.getEME2000(), angles, AngleType.RADEC, distance);
            angularPos.setName(noradId);
            angularPos.setDate(date);

            geo.add(angularPos);            
        }
        return geo;
    }

    public static List<AngularDirection> callStripeScanGeoFrame(Sensor sensor, Stripe scan, 
                                                                int numExpo, AbsoluteDate start, 
                                                                AngularDirection sensorPointing) {

        // Settings 
        double preparation = 6.;

        // Declare output
        List<AngularDirection> scheduleGeo = new ArrayList<AngularDirection>();

        // Re-compute allocation period to slew from current sensor position towards stripe position
        AngularDirection newSensorPointing = 
            scan.getPosField(0)
                .transformReference(sensorPointing.getFrame(), start, 
                                    sensorPointing.getAngleType());
        //newSensorPointing.setDate(start);
        double actualSlewT = 
                sensor.computeRepositionT(sensorPointing, newSensorPointing, true);
        double allocation = actualSlewT;

        // reposition to scan stripe
        AbsoluteDate arriveAtStripe = start.shiftedBy(allocation + sensor.getSettlingT() + preparation);
        AbsoluteDate nextPointing = arriveAtStripe.shiftedBy(sensor.getExposureT()/2.);
        newSensorPointing.setDate(nextPointing);

        // reposition inside scan stripe
        double reposDuration = scan.getReposInStripeT();

        // Set pointing direction and target date
        for (int i=0; i<scan.getNumDecFields(); i++) {
            
            for (int j=0; j<numExpo; j++) {
                AngularDirection decField = scan.getPosField(i);
                decField.setDate(nextPointing);
 
                // In same declination field 
                scheduleGeo.add(decField);
                nextPointing = nextPointing.shiftedBy(sensor.getExposureT() + sensor.getReadoutT());
            }
           
            // last measurement does not require extra time for read out (already covered by repos)
            // Compute time stamp for new dec field
            nextPointing = nextPointing.shiftedBy(reposDuration - sensor.getReadoutT() 
                                                    + preparation);
        }
        return scheduleGeo;
    }

    @Test
    public void iodTest() {
        IodGooding gooding = new IodGooding(Constants.WGS84_EARTH_MU);

        // Station
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),      // Geodetic latitude
                                              FastMath.toRadians(-37.),      // Longitude
                                     0.);                           // in [m]
        TopocentricFrame topo = new TopocentricFrame(earth, pos, "Topocentric");

        // Set up Gooding
        double[] measNoise = new double[]{FastMath.pow(1./206265., 2),
                                          FastMath.pow(1./206265., 2)};
        double[] radec1 = new double[]{FastMath.toRadians(192.7055590353602), 
                                       FastMath.toRadians(-1.7132729865117795)};
        double[] radec3 = new double[]{FastMath.toRadians(192.82709253672132), 
                                       FastMath.toRadians(-1.744012461398498)};
        double[] radec5 = new double[]{FastMath.toRadians(192.9486303768182), 
                                       FastMath.toRadians(-1.7747441871835372)};
        
        GroundStation sensor = new GroundStation(topo);
        AngularRaDec angle1 = new AngularRaDec(sensor, 
                                               FramesFactory.getEME2000(), 
                                               new AbsoluteDate("2025-03-24T22:05:10.000Z", 
                                                                TimeScalesFactory.getUTC()), 
                                               radec1, measNoise, new double[]{1., 1.}, 
                                               new ObservableSatellite(0));
        AngularRaDec angle2 = new AngularRaDec(sensor, 
                                               FramesFactory.getEME2000(), 
                                               new AbsoluteDate("2025-03-24T22:05:40.000Z", 
                                                                TimeScalesFactory.getUTC()), 
                                               radec3, measNoise, new double[]{1., 1.}, 
                                               new ObservableSatellite(0));
        AngularRaDec angle3 = new AngularRaDec(sensor, 
                                               FramesFactory.getEME2000(), 
                                               new AbsoluteDate("2025-03-24T22:06:10.000Z", 
                                                                TimeScalesFactory.getUTC()), 
                                               radec5, measNoise, new double[]{1., 1.}, 
                                               new ObservableSatellite(0));
                                               
    }
}
