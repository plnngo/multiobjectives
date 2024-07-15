package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.orekit.frames.TopocentricFrame;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;
import sensortasking.stripescanning.Stripe;

@Getter
public class SearchObjective implements Objective{

    TopocentricFrame stationHorizon;

    Stripe scan;

    int numExpo;

    Sensor sensor;

    List<AngularDirection> schedule;

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
    public AngularDirection setMicroAction(AbsoluteDate current) {

        List<AngularDirection> stripe = callStripeScan(current);
        return stripe.get(0);
    }


    private List<AngularDirection> callStripeScan(AbsoluteDate start) {

        // Declare output
        List<AngularDirection> schedule = new ArrayList<AngularDirection>();

        // reposition to scan stripe
        AbsoluteDate arriveAtStripe = start.shiftedBy(allocation + settling + preparation);
        AbsoluteDate nextPointing = arriveAtStripe.shiftedBy(exposure/2.);

        // reposition inside scan stripe
        double reposDuration = this.scan.getReposInStripeT();

        // Set pointing direction and target date
        for (int i=0; i<scan.getNumDecFields(); i++) {
            
            for (int j=0; j<this.numExpo; j++) {
                AngularDirection decField = scan.getPosField(i);
                
                // In same declination field 
                decField.setDate(nextPointing);
                schedule.add(decField);
                nextPointing = nextPointing.shiftedBy(exposure/2. + sensor.getReadoutT());
            }
           
            // last measurement does not require extra time for read out (already covered by repos)
            nextPointing = nextPointing.shiftedBy(reposDuration - sensor.getReadoutT());
        }

/*         double scanDuration = this.scan.getStripeT(this.numExpo);
        double decFieldDuration = 
            this.numExpo * sensor.getExposureT() + (this.numExpo - 1) * sensor.getReadoutT(); */
        
        this.schedule = schedule;
        return schedule;
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        AbsoluteDate lastMeas = this.schedule.get(this.schedule.size()-1).getDate();
        AbsoluteDate firstMeas = this.schedule.get(0).getDate();
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
