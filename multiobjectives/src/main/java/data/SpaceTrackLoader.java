package data;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.CookieHandler;
import java.net.CookieManager;
import java.net.CookiePolicy;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;
import java.util.AbstractMap.SimpleEntry;

import javax.net.ssl.HttpsURLConnection;

import org.orekit.data.DataSource;
import org.orekit.files.ccsds.ndm.ParserBuilder;
import org.orekit.files.ccsds.ndm.odm.omm.OmmParser;
import org.orekit.utils.Constants;

public class SpaceTrackLoader {

    /**
     * Build query of list of satellites either with their latest TLE or with all TLEs in a 
     * predefined time range. Query shall then be forwarded to space-track.
     * 
     * @param noradId                       Array of Norad IDs.
     * @param sortByEpoch                   True, if history shall be sorted by epoch. 
     *                                      False, if it shall be sorted by Norad ID.
     * @param startDate                     Start date in the format YYYY-MM-DD, if null then 
     *                                      retrieve only latest TLE.
     * @param endDate                       End date in the format YYYY-MM-DD, if null then 
     *                                      retrieve only latest TLE.
     * @return                              Generated query.
     */
    public static String buildTleQuery(final int[] noradId, final boolean sortByEpoch,
                                   final String startDate, final String endDate) {
            
        // Build Norad ID query (remove last comma)
        String noradIdQuery = "";
        for (int i = 0; i<noradId.length; i++) {
            noradIdQuery = noradIdQuery + String.valueOf(noradId[i]) + ",";
        }
        noradIdQuery = noradIdQuery.substring(0, noradIdQuery.length() - 1);

        if(Objects.isNull(startDate) || Objects.isNull(endDate)) {

            // Only download the latest TLE from the list of objects
            String query = 
            "/basicspacedata/query/class/gp_history/NORAD_CAT_ID/"
            + noradIdQuery
            + "/orderby/TLE_LINE1 ASC/format/tle";
            return query;
        }

        // Build Epoch query
        String epochQuery = startDate + "--" + endDate;

        // Build sorting query
        String sortingQuery = "";
        if (sortByEpoch) {
            sortingQuery = "EPOCH%20ASC";
        } else {
            sortingQuery = "TLE_LINE1%20ASC";
        }

        String query = 
            "/basicspacedata/query/class/gp_history/NORAD_CAT_ID/"
            + noradIdQuery
            + "/orderby/"
            + sortingQuery
            + "/EPOCH/"
            + epochQuery
            + "/"
            + "format/tle";
        
        return query;
    }
    
    public static String buildOmmQuery(final int[] noradId, final boolean sortByEpoch, 
                                     final String limitMeanMotion, final String limitEcc, 
                                     final String limitInc, final String limitRaan, 
                                     final String limitArgPerigee, final String limitMeanAnomaly,
                                     final String limitEpoch, final String format){

        String query = "/basicspacedata/query/class/gp";

        // Build Norad ID query (remove last comma)
        if (Objects.nonNull(noradId) && noradId.length != 0) {
            String noradIdQuery = "";
            for (int i = 0; i<noradId.length; i++) {
                noradIdQuery = noradIdQuery + String.valueOf(noradId[i]) + ",";
            }
            noradIdQuery = noradIdQuery.substring(0, noradIdQuery.length() - 1);
            query += "/NORAD_CAT_ID/" + noradIdQuery;
        }

        // Build sorting query
        String sortingQuery = "";
        if (sortByEpoch) {
            sortingQuery = "EPOCH%20ASC";
        } else {
            sortingQuery = "TLE_LINE1%20ASC";
        }
        query += "/orderby/" + sortingQuery;

        // Build time constrain query
        if (!Objects.isNull(limitEpoch) && !limitEpoch.isEmpty()) {
            query += "/EPOCH/" + limitEpoch;
        }

        // Constrain mean motion
        if (!Objects.isNull(limitMeanMotion) && !limitMeanMotion.isEmpty()) {
            query += "/MEAN_MOTION/" + limitMeanMotion;
        }

        // Constrain eccentricity
        if (!Objects.isNull(limitEcc) && !limitEcc.isEmpty()) {
            query += "/ECCENTRICITY/" + limitEcc;
        }

        // Constrain inclination
        if (!Objects.isNull(limitInc) && !limitInc.isEmpty()) {
            query += "/INCLINATION/" + limitInc;
        }

        // Constrain Raan
        if (!Objects.isNull(limitRaan) && !limitRaan.isEmpty()) {
            query += "/RA_OF_ASC_NODE/" + limitRaan;
        }

        // Constrain argument of perigee
        if (!Objects.isNull(limitArgPerigee) && !limitArgPerigee.isEmpty()) {
            query += "/ARG_OF_PERICENTER/" + limitArgPerigee;
        }

        // Constrain mean anomaly
        if (!Objects.isNull(limitMeanAnomaly) && !limitMeanAnomaly.isEmpty()) {
            query += "/MEAN_ANOMALY/" + limitMeanAnomaly;
        }

        // Build format query
        query +="/format/" + format;

        return query;
    }

     /**
     * Connect to space-track in order to download TLEs according to {@link #query}}.           
     */
    public static void download(final String query, final Path filepath, final String username, final String password) {

        try {

            final String baseURL = "https://www.space-track.org";
            final String authPath = "/ajaxauth/login";

            final CookieManager manager = new CookieManager();
            manager.setCookiePolicy(CookiePolicy.ACCEPT_ALL);
            CookieHandler.setDefault(manager);

            URL url = new URL(baseURL + authPath);
            final HttpsURLConnection conn = (HttpsURLConnection) url.openConnection();
            conn.setDoOutput(true);
            conn.setRequestMethod("POST");
            
            // Set proxy if FHR network blocks access
            OutputStream os;
            final String input = "identity=" + username + "&password=" + password;
            try {
                os = conn.getOutputStream();
                os.write(input.getBytes());
                os.flush();
                os.close();

            } catch(ConnectException exp) {
                os = conn.getOutputStream();
                os.write(input.getBytes());
                os.flush();
                os.close();
            }
            
            BufferedReader br = new BufferedReader(new InputStreamReader((conn.getInputStream())));

            String output;
            System.out.print("Downloading TLE from space-track.org... ");

            url = new URL(baseURL + query);
            br = new BufferedReader(new InputStreamReader((url.openStream())));

            try (BufferedWriter writer = Files.newBufferedWriter(filepath, StandardCharsets.UTF_8)) {

                while ((output = br.readLine()) != null) {
                    writer.write(output + "\n");
                }

            } catch (final IOException x) {
                System.err.format("IOException: %s%n", x);
            }

            url = new URL(baseURL + "/ajaxauth/logout");
            br = new BufferedReader(new InputStreamReader((url.openStream())));
            conn.disconnect();

            System.out.println("finished.");

        } catch (final Exception e) {

            e.printStackTrace();
        }
    }

    public void parseOmm(String pathOmm){

        final DataSource source = new DataSource(pathOmm, () -> getClass().getResourceAsStream(pathOmm));
        OmmParser parser = new ParserBuilder().withMu(Constants.IERS2010_EARTH_MU).buildOmmParser();
        parser.parseMessage(source);
    }

    public static SimpleEntry<String, String> requestUsernameAndPassword() throws IOException{

        System.out.println("Enter space-track username: ");
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));

        String username = reader.readLine();
        System.out.println("Enter space-track password: ");
        String password = reader.readLine();


        return new SimpleEntry<String,String>(username, password);
    }

    public static void main(String[] args) throws IOException {
        System.out.println(System.getProperty("user.dir"));
        SpaceTrackLoader test = new SpaceTrackLoader();
        
        test.parseOmm("TestFile.xml");
    }
}
