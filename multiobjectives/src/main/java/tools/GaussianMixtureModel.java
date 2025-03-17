package tools;

import java.util.ArrayList;
import java.util.List;

import lombok.Getter;

@Getter
public class GaussianMixtureModel {

    double[] weights;

    double[][] means;

    List<double[][]> P;

    public GaussianMixtureModel(double[] weights, double[][] means, List<double[][]> P) {

        // Check dimensions
        int N = weights.length;
        if (N != means.length) {
            throw new IllegalArgumentException("Weights and means do not have the same size");
        } else if (P.size() != N) {
            throw new IllegalArgumentException("P and weights do not have the same size");
        }

        // Initialise global variables
        this.weights = new double[weights.length];
        this.means = new double[means.length][means[0].length];
        this.P = new ArrayList<double[][]>();

        for (int i=0; i<N; i++) {
            this.weights[i] = weights[i];
            
            // Deep copy means
            for (int j=0; j<means[0].length; j++) {
                this.means[i][j] = means[i][j];
            }

            // Deep copy P
            double[][] Pcopy = P.get(i);
            double[][] Pi = new double[Pcopy.length][Pcopy[0].length];
            for (int m=0; m<Pi.length; m++) {
                for (int n=0; n<Pi[0].length; n++) {
                    Pi[m][n] = Pcopy[m][n];
                }
            }
            this.P.add(Pi);
        }

    }
    
}
