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
    public void testSortingByFirstEntry() {

        // Set input
        List<double[]> vectors = new ArrayList<double[]>(); 
        vectors.add(new double[]{8, 9, 2});
        vectors.add(new double[]{9, 4, 2});
        vectors.add(new double[]{5, 3, 2});
        vectors.add(new double[]{10, 7, 3});
        vectors.add(new double[]{9, 7, 5});
        vectors.add(new double[]{4, 3, 5});
        vectors.add(new double[]{9, 8, 3});
        vectors.add(new double[]{1, 7, 10});
        vectors.add(new double[]{4, 4, 5});
        vectors.add(new double[]{1, 9, 5});

        OptimisingVector test = new OptimisingVector(vectors);
        double[] firstActual = test.getAll().get(0);
        for(int i=0; i<firstActual.length; i++){
            Assert.assertEquals(vectors.get(7)[i], firstActual[i], 1e-16);
        }

    }
}
