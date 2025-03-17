package tools;

public class MatrixTools {

    public static double[][] concatenateColumns(double[][] A, double[][] B) {
        int rows = A.length;  // Assumes A and B have the same number of rows
        int colsA = A[0].length, colsB = B[0].length;

        double[][] result = new double[rows][colsA + colsB];

        for (int i = 0; i < rows; i++) {
            System.arraycopy(A[i], 0, result[i], 0, colsA); // Copy A's row
            System.arraycopy(B[i], 0, result[i], colsA, colsB); // Copy B's row
        }

        return result;
    }

    public static double[][] subtractEachColumnByVec(double[][] matrix, double[] vec) {
        int rows = matrix.length;
        int cols = matrix[0].length;

        if (vec.length != rows) {
            throw new IllegalArgumentException("Vector length must match the number of rows "
                                                + "in the matrix.");
        }

        double[][] result = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = matrix[i][j] - vec[i];  
            }
        }

        return result;
    }
} 
