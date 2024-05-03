package tools;
import java.util.Random;

public class WeightedRandomNumberPicker {
    

    
    // Method to pick a random number with weighted probability
    public static int pickNumber(int[] numbers, double[] weights) {
        // Validate input arrays
        if (numbers.length != weights.length || numbers.length == 0) {
            throw new IllegalArgumentException("Invalid input arrays");
        }
        
        // Calculate total weight
        double totalWeight = 0;
        for (double weight : weights) {
            totalWeight += weight;
        }
        
        // Generate a random number within the range of total weight
        Random random = new Random();
        double randomNumber = random.nextDouble() * totalWeight;
        
        // Pick the corresponding number based on weighted probability
        double cumulativeWeight = 0;
        for (int i = 0; i < numbers.length; i++) {
            cumulativeWeight += weights[i];
            if (randomNumber < cumulativeWeight) {
                return numbers[i];
            }
        }
        
        // This should never be reached, but to satisfy compiler
        return numbers[numbers.length - 1];
    }
    
    public static void main(String[] args) {
        // Example input arrays
        int[] numbers = {1, 2, 3, 4, 5};
        double[] weights = {0.1, 0.2, 0.4, 0.2, 0.1}; // Adjust weights as needed
        int[] counter = new int[]{0, 0, 0, 0, 0};
        
        // Pick a random number with weighted probability
        for (int i=0; i<1000000; i++) {
            int randomWeightedNumber = pickNumber(numbers, weights);
            counter[randomWeightedNumber-1] += 1;
        }
        for(int i=0; i<counter.length; i++){
            System.out.println("Number "+ (i+1) + " occured " + counter[i] + " times");
        }
    }
}


