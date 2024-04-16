package data;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;

import org.junit.Before;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;


public class SpaceTrackLoaderTest {

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
    public void testBuildOmmQueryLimitNoradIdAndEpoch(){

        // Set input of the query
        int[] noradId = new int[]{25544, 38101};
        boolean sortByEpoch = true;
        String limitInc = "";
        String limitEpoch = ">now-30";
        String format = "xml";

        String actualQuery = 
            SpaceTrackLoader.buildOmmQuery(noradId, sortByEpoch, null, null, 
                                           limitInc, null, null, null, limitEpoch, format);

        // Build expected query string
        String expectedQuery = "/basicspacedata/query/class/gp" +  "/NORAD_CAT_ID/" 
                                + Integer.toString(noradId[0]) + "," + Integer.toString(noradId[1])
                                + "/orderby/" + "EPOCH%20ASC"
                                + "/EPOCH/" + limitEpoch 
                                + "/format/" + format;

        // Assert
        org.junit.Assert.assertEquals(expectedQuery, actualQuery);
    } 

    @Test
    public void testBuildOmmQueryLimitKeplerElements(){
        boolean sortByEpoch = true;
        String limitInc = "<5";
        String limitEcc = "<0.01";
        String limitMeanMotion = "0.99--1.01";
        String limitRaan = ">45";
        String limitMeanAnomaly = "80--100";
        String format = "xml";

        String actualQuery = 
        SpaceTrackLoader.buildOmmQuery(null, sortByEpoch, limitMeanMotion, limitEcc, limitInc, 
                                       limitRaan, null, limitMeanAnomaly, null, format);

        String expectedQuery = "/basicspacedata/query/class/gp/orderby/EPOCH%20ASC"
                                + "/MEAN_MOTION/" + limitMeanMotion
                                + "/ECCENTRICITY/" + limitEcc
                                +"/INCLINATION/" + limitInc
                                + "/RA_OF_ASC_NODE/" + limitRaan
                                + "/MEAN_ANOMALY/" + limitMeanAnomaly 
                                + "/format/xml";

        // Assert
        org.junit.Assert.assertEquals(expectedQuery, actualQuery);
    }

    @Test
    public void testDownload() throws IOException{
        String query = "/basicspacedata/query/class/gp/MEAN_MOTION/0.99--1.01/INCLINATION/%3C5/"
                        + "ECCENTRICITY/%3C0.01/RA_OF_ASC_NODE///%3E45/MEAN_ANOMALY/80--100/"
                        + "orderby/EPOCH%20asc/format/tle";

        Path filepath = FileSystems.getDefault().getPath("src/test/java/output/TestFile.tle");

        String username = "p.l.n.ngo@tudelft.nl";
        String password = "Z317d1l474710n!";
        SpaceTrackLoader.download(query, filepath, username, password);
    }

    @Test
    public void testDownload2(){
        String query = "/basicspacedata/query/class/gp/orderby/NORAD_CAT_ID,EPOCH/format/tle";
        Path filepath = FileSystems.getDefault().getPath("src/test/java/output/Catalogue_16042024.tle");

        String username = "p.l.n.ngo@tudelft.nl";
        String password = "Z317d1l474710n!";
        SpaceTrackLoader.download(query, filepath, username, password);
    }

}
