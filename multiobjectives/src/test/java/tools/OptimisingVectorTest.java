package tools;

import java.io.File;
import java.util.List;
import java.util.ArrayList;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;

public class OptimisingVectorTest {

    public List<double[]> vectors = new ArrayList<double[]>();
    
    @Before
    public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // set input 
        this.vectors.add(new double[]{8, 9, 2});
        this.vectors.add(new double[]{9, 4, 2});
        this.vectors.add(new double[]{5, 3, 2});
        this.vectors.add(new double[]{10, 7, 3});
        this.vectors.add(new double[]{9, 7, 5});
        this.vectors.add(new double[]{4, 3, 5});
        this.vectors.add(new double[]{9, 8, 3});
        this.vectors.add(new double[]{1, 7, 10});
        this.vectors.add(new double[]{4, 4, 5});
        this.vectors.add(new double[]{1, 9, 5});
    }

    @Test
    public void testSortingByFirstEntry() {
       
        OptimisingVector test = new OptimisingVector(this.vectors, 0);
        double[] firstActual = test.getAll().get(0);
        for(int i=0; i<firstActual.length; i++){
            Assert.assertEquals(vectors.get(7)[i], firstActual[i], 1e-16);
        }
    }

    @Test
    public void testGetDominatingVecs(){
        OptimisingVector test = new OptimisingVector(this.vectors, 0);
        List<double[]> out = 
            test.getDominatingVecs(new double[]{7, 8, 4}, new boolean[]{false, false, false}, 0);
        Assert.assertEquals(1, out.size());
        for(int i=0; i<out.get(0).length; i++) {
            Assert.assertEquals(vectors.get(2)[i], out.get(0)[i], 1e-16);
        }

        out = test.getDominatingVecs(new double[]{7, 8, 4}, new boolean[]{true, true, true}, 0);
        Assert.assertEquals(0, out.size());
    }

    @Test
    public void testGetDominatingVecs2(){
        List<double[]> testSet = new ArrayList<double[]>();
        testSet.add(new double[]{0.3, 0.3, 0.3});
        testSet.add(new double[]{0.7, 0.7, 0.3});

        OptimisingVector test = new OptimisingVector(testSet, 0);
        List<double[]> out = 
            test.getDominatingVecs(new double[]{0.7, 0.3, 0.3}, new boolean[]{false, false, false}, 0);
        Assert.assertEquals(1, out.size());
        for(int i=0; i<out.get(0).length; i++) {
            Assert.assertEquals(testSet.get(0)[i], out.get(0)[i], 1e-16);
        }
    }

    @Test
    public void testGetDominatingVecs3() {
        List<double[]> testVec = new ArrayList<double[]>();
        testVec.add(new double[]{0.0, -0.008923});
        testVec.add(new double[]{0.0, 1.270499});
        testVec.add(new double[]{0.0, -0.0});
        testVec.add(new double[]{0.0, 1.721756});

        OptimisingVector test = new OptimisingVector(testVec, 0);
        double[] toCompare = new double[]{0.0, 0.618239};
        List<double[]> out = test.getDominatingVecs(toCompare, new boolean[]{true, true}, 0);
        Assert.assertEquals(2, out.size());
        for(int i=0; i<out.get(0).length; i++) {
            Assert.assertEquals(testVec.get(1)[i], out.get(1)[i], 1e-16);
            Assert.assertEquals(testVec.get(3)[i], out.get(0)[i], 1e-16);
        }        
    }
}
