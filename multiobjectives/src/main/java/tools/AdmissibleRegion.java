package tools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AdmissibleRegion {

    /** Range values. */
    List<Double> rho_output = new ArrayList<Double>();

    /** Range-rate values. */
    List<Double> drho_output = new ArrayList<Double>();

    /** AR boundry. Range and range-rate values organised as a map. */
    Map<Double, double[]> drho_dict = new HashMap<Double, double[]>();

    /** Range values regarding a-CAR (constrained in semi-major axis). */
    List<Double> rho_a_all = new ArrayList<Double>();

    /** Range-rate values regarding a-CAR (constrained in semi-major axis) */
    List<Double> drho_a_all = new ArrayList<Double>();

    /** Range values regarding e-CAR (constrained in eccentricity). */
    List<Double> rho_e_all = new ArrayList<Double>();

    /** Range-rate values regarding e-CAR (constrained in eccentricity) */
    List<Double> drho_e_all = new ArrayList<Double>();


    public AdmissibleRegion(List<Double> ranges, List<Double> rangeRates, Map<Double, 
                            double[]> boundary, List<Double> rangesConstrainedA, 
                            List<Double> rangeRatesConstrainedA, List<Double> rangesConstrainedEcc, 
                            List<Double> rangeRatesConstrainedEcc) {

        // Copy ranges and range-rates defining CAR
        for (Map.Entry<Double, double[]> entry : boundary.entrySet()) {
            this.drho_dict.put(entry.getKey(), entry.getValue().clone());
        }

    }
    
}
