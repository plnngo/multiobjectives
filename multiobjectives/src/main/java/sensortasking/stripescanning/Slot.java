package sensortasking.stripescanning;

import org.orekit.time.AbsoluteDate;

import sensortasking.mcts.Sensor;

public class Slot {

    /** Start epoch of slot time interval. */
    final private AbsoluteDate start;

    /** End epoch of slot time interval. */
    private AbsoluteDate end;

    /** Stripe that is scanned with slot time interval. */
    private Stripe stripe;

    /** Repositioning time within stripe from one field to the next one. */
    private double reposInStripeT;

    /** Time duration dedicated for one declination field. */
    private double declinationFieldT;
    
    /**
     * Constructor to create a slot defined by its time interval. No scanning stripe is assigned 
     * to this slot.
     * 
     * @param startEpoch            Start epoch of the time interval.
     * @param endEpoch              End epoch of the time interval.
     */
    public Slot(AbsoluteDate startEpoch, AbsoluteDate endEpoch) {
        this.start = startEpoch;
        this.end = endEpoch;
    }

    /**
     * Constructor to create a slot defined by its time interval with an assigned scan stripe. 
     * 
     * @param startEpoch            Start epoch of the time interval.
     * @param endEpoch              End epoch of the time interval.
     * @param numExpos              Number of exposures to one declination field.
     */
    public Slot(AbsoluteDate startEpoch, AbsoluteDate endEpoch, Stripe stripe, int numExpos) {
        this.start = startEpoch;
        this.end = endEpoch;
        this.stripe = stripe;
        stripe.getStripeT(numExpos);
        this.reposInStripeT = stripe.getReposInStripeT();
        Sensor sensor = stripe.getSensor();
        this.declinationFieldT = numExpos * sensor.getExposureT() + (numExpos - 1) * sensor.getReadoutT();
    }

    /** 
     * Set scanning stripe.
     * 
     * @param scanStripe            Assigned scanning stripe.
     * @param numExpos              Number of exposures to one declination field.
     */
    public void setStripe(Stripe scanStripe, int numExpos) {
        this.stripe = scanStripe;
        scanStripe.getStripeT(numExpos);
        this.reposInStripeT = scanStripe.getReposInStripeT();
        Sensor sensor = scanStripe.getSensor();
        this.declinationFieldT = numExpos * sensor.getExposureT() + (numExpos - 1) * sensor.getReadoutT();
    }

    /**
     * Get start date.
     * 
     * @return                      Start epoch of slot.
     */
    public AbsoluteDate getStart(){
        return this.start;
    }

    /**
     * Get end date.
     * 
     * @return                      End epoch of slot.
     */
    public AbsoluteDate getEnd(){
        return this.end;
    }

    /**
     * Get scan stripe.
     * 
     * @return                      Scan stripe.
     */
    public Stripe getStripe(){
        return this.stripe;
    }

    /**
     * Get time duration it takes to reposition scan stripe.
     * 
     * @return                      Time duration it takes to reposition scan stripe in [s].
     */
    public double getReposInStripeT(int numExpos){
        return this.reposInStripeT;
    }

    /**
     * Get time duration the sensor spends on one declination field.
     * 
     * @return                      Time duration in one declination field in [s] before repositioning.
     */
    public double getDeclinationFieldT() {
        return this.declinationFieldT;
    }

}
