package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;
import sensortasking.stripescanning.Stripe;

@Getter
public class SearchObjective implements Objective{

    TopocentricFrame stationHorizon;

    Stripe scan;

    int numExpo;

    Sensor sensor;

    List<AngularDirection> scheduleGeocentric;

    List<AngularDirection> scheduleTopocentric;

    // TODO implement as sensor object
    static double readout = 7.;
    static double exposure = 8.;
    static double allocation = 60.;
    static double settling = 30.;
    static double preparation = 6.;

    public SearchObjective(TopocentricFrame horizon, Stripe scan, int numExpo, Sensor sensor) {
        this.stationHorizon = horizon;
        this.scan = scan;
        this.numExpo = numExpo;
        this.sensor = sensor;
    }


    @Override
    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing) {

        List<AngularDirection> stripe = callStripeScanTopoFrame(current);
        return stripe.get(0);
    }


    private List<AngularDirection> callStripeScanTopoFrame(AbsoluteDate start) {

        // Inertial frame
        Frame j2000 = FramesFactory.getEME2000();

        // Declare output
        List<AngularDirection> scheduleTopo = new ArrayList<AngularDirection>();
        List<AngularDirection> scheduleGeo = new ArrayList<AngularDirection>();


        // reposition to scan stripe
        AbsoluteDate arriveAtStripe = start.shiftedBy(allocation + settling + preparation);
        AbsoluteDate nextPointing = arriveAtStripe.shiftedBy(exposure/2.);

        // reposition inside scan stripe
        double reposDuration = this.scan.getReposInStripeT();

        // Set pointing direction and target date
        for (int i=0; i<scan.getNumDecFields(); i++) {
            
            for (int j=0; j<this.numExpo; j++) {
                AngularDirection decField = scan.getPosField(i);
                decField.setDate(nextPointing);

                // Transform direction into topocentric inertial frame
                Transform horizonToEci = this.stationHorizon.getTransformTo(j2000, nextPointing);  
                Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
                Transform eciToTopo = new Transform(nextPointing, coordinatesStationEci.negate());
                Frame topocentric = new Frame(j2000, eciToTopo, "Topocentric", true);
                AngularDirection decFieldTopo = 
                    decField.transformReference(topocentric, nextPointing, AngleType.RADEC);
                decFieldTopo.setDate(nextPointing);
                
                // In same declination field 
                scheduleTopo.add(decFieldTopo);
                scheduleGeo.add(decField);
                nextPointing = nextPointing.shiftedBy(exposure + sensor.getReadoutT());
            }
           
            // last measurement does not require extra time for read out (already covered by repos)
            nextPointing = nextPointing.shiftedBy(reposDuration - sensor.getReadoutT());
        }

/*         double scanDuration = this.scan.getStripeT(this.numExpo);
        double decFieldDuration = 
            this.numExpo * sensor.getExposureT() + (this.numExpo - 1) * sensor.getReadoutT(); */
        
        this.scheduleTopocentric = scheduleTopo;
        this.scheduleGeocentric = scheduleGeo;
        return scheduleTopo;
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        AbsoluteDate lastMeas = this.scheduleTopocentric.get(this.scheduleTopocentric.size()-1).getDate();
        AbsoluteDate firstMeas = this.scheduleTopocentric.get(0).getDate();
        double stripeT = lastMeas.durationFrom(firstMeas);
        double taskDuration = allocation + settling + preparation + sensor.getExposureT() 
                                + stripeT + sensor.getReadoutT();
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current.shiftedBy(taskDuration)};
        return interval;
    }

    @Override
    public List<ObservedObject> propagateOutcome() {

       /*  // Fake data
        StateVector state = new StateVector();
        state.setX(100.);
        state.setY(200.);
        state.setZ(300.);

        CartesianCovariance cov = new CartesianCovariance(null);
        for (int pos=0; pos<3; pos++) {
            cov.setCovarianceMatrixEntry(pos, pos, 10.);
            cov.setCovarianceMatrixEntry(pos+3, pos+3, 100.);
        }
        
        ObservedObject obj = new ObservedObject(0123, state, cov, new AbsoluteDate(), FramesFactory.getTEME());
        List<ObservedObject> out = new ArrayList<ObservedObject>();
        out.add(obj); */
        return null;
    }
}
