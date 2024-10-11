package tools;

import lombok.Getter;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.ArrayList;

@Getter
public class OptimisingVector {

    protected List<double[]> all;

    public OptimisingVector(List<double[]> vectors, int dim) {
        if(vectors.size()>0) {
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
                    Double d1 = o1[dim];
                    Double d2 = o2[dim];
                    return d1.compareTo(d2);
                }
            });

            List<double[]> sorted = Arrays.asList(allVec);
            this.all = sorted;
        }
        else {
            this.all = new ArrayList<double[]>();
        }
    }

    public void addVec(double[] toAdd) {

        // Add new vector in the right order to the vector sequences according to 1st entry
        for(int i=0; i<this.all.size(); i++) {
            if(all.get(i)[0]>toAdd[0]) {
                this.all.add(i, toAdd);
                break;
            } else if(i==this.all.size()-1) {
                this.all.add(toAdd);
            }
        }
    }
    public List<double[]> getDominatingVecs(double[] toCompare, boolean[] order, int dim) {

        if(toCompare.length != order.length) {
            throw new IllegalArgumentException("Vector and order instructions" 
                                                + " do not have the same dimensions");
        } 
        if (dim == toCompare.length) {
            return this.all;
        } else {

            // Order vector of sequence according to instruction
            List<double[]> dominating = new ArrayList<double[]>();
            if(order[dim]) {
                // descending order = vector entry dominates by maximising
                for(int entry=0; entry<this.all.size(); entry++) {
                    int endIndex = this.all.size() - 1;
                    if(this.all.get(endIndex - entry)[dim] > toCompare[dim]) {
                        dominating.add(this.all.get(endIndex - entry));
                    } else {
                        break;
                    }
                }
                
            } else {
                // ascending order = vector entry dominates by minimising
                for(int entry=0; entry<this.all.size(); entry++) {
                    if (this.all.get(entry)[dim] < toCompare[dim]) {
                        dominating.add(this.all.get(entry));
                    } else {
                        break;
                    }
                }
            }
            dim++;
            if(dim<toCompare.length){
                OptimisingVector ov = new OptimisingVector(dominating, dim);
                return ov.getDominatingVecs(toCompare, order, dim);
            } else {
                return dominating;
            }
        } 
    }

}
