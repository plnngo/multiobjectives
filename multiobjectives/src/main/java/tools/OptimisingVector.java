package tools;

import java.util.List;

import lombok.Getter;

import java.util.Arrays;
import java.util.Comparator;

@Getter
public class OptimisingVector {

    protected List<double[]> all;

    public OptimisingVector(List<double[]> vectors) {
        double[][] allVec = new double[vectors.size()][vectors.get(0).length];

        // Fill multi dimensional array with vector entries
        for(int i=0; i<vectors.size(); i++){
            double[] vec = vectors.get(i);
            for(int j=0; j<vec.length; j++) {
                allVec[i][j] = vec[j]; 
            }
        }

        // Sort array by first entries of vector
        Arrays.sort(allVec, new Comparator<double[]>() {

            @Override
            public int compare(double[] o1, double[] o2) {
                Double d1 = o1[0];
                Double d2 = o2[0];
                return d1.compareTo(d2);
            }
        });

        List<double[]> sorted = Arrays.asList(allVec);
        this.all = sorted;
    }


}
