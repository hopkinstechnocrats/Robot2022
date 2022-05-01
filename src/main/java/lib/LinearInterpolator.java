package lib;

import java.util.HashMap;

public class LinearInterpolator {

    private final HashMap<Double, Double> table;


    public LinearInterpolator() {
        table = new HashMap<Double, Double>();
    }

    // Add a row to the interpolation table
    public void put(double key, double value) {
        table.put(key, value);
    }

    // Interpolates for input value of "key"
    public double get(double key) {
        // finding the minimum key
        Double min = Double.MAX_VALUE;

        for (Double j : table.keySet()) {
            if (j < min) {
                min = j;
            }
        }

        // finding the maximum key
        Double max = Double.MIN_VALUE;

        for (Double k : table.keySet()) {
            if (k > max) {
                max = k;
            }
        }

        Double valueAbove = max;
        Double valueBelow = min;

        for (Double i : table.keySet()) {
            if (i >= valueBelow && i <= key) {
                valueBelow = i;
            }
            if (i <= valueAbove && i >= key) {
                valueAbove = i;
            }
        }

        Double x1 = valueBelow;
        Double x2 = valueAbove;
        Double y1 = table.get(valueBelow);
        Double y2 = table.get(valueAbove);

        if (x1 == x2) {
            return table.get(x1);
        }

        return ((y2 - y1) / (x2 - x1)) * (key - x1) + y1;

    }
}