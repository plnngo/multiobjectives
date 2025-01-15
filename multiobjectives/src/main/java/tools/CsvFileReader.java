package tools;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import com.opencsv.CSVReader;

public class CsvFileReader {

    public static void main(String[] args) throws FileNotFoundException, IOException {
        Map<String[], Integer> statistics = new LinkedHashMap<String[], Integer>();
        try (CSVReader reader = new CSVReader(new FileReader("multiobjectives\\Tuples_11.csv"))) {
            List<String[]> r = reader.readAll();
            for(String[] x : r) {
                boolean solutionRegistered = false;
                for (Map.Entry<String[], Integer> entry : statistics.entrySet()) {
                    //solutionRegistered = entry.getKey().equals(x);
                    int i =0;
                    while(i<x.length) {
                        //if(! x[i].equals(entry.getKey()[i])) {
                        if(!Objects.equals(x[i], entry.getKey()[i])){
                            solutionRegistered = false;
                            break;
                        }
                        i++;
                    }
                    if(i==x.length) {
                        solutionRegistered = true;
                        statistics.put(entry.getKey(), entry.getValue() + 1);
                        break;
                    }
                    
                }
                if(!solutionRegistered) {
                    statistics.put(x, 1);
                }
            }
        }
        System.out.println("Number of unique solutions: " + statistics.size());
    }   
}