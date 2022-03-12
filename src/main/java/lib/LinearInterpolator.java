package lib;

import java.util.HashMap;

public class LinearInterpolator {

    private HashMap<Double, Double> table;


    public LinearInterpolator() {
        table = new HashMap<Double, Double>();
    }

    // Add a row to the interpolation table
    public void put(Double key, Double value) {
        table.put(key, value);
    }

    // Interpolates for input value of "key"
    public Double get(Double key) {
        
        Double valueAbove = Double.MAX_VALUE;
        Double valueBelow = Double.MIN_VALUE;

        for (Double i : table.keySet()) {
            if (i > valueBelow && i < key) {
                valueBelow = i;
            }
            if (i < valueAbove && i > key) {
                valueAbove = i;
            }
        }

        Double x1 = valueBelow;
        Double x2 = valueAbove;
        Double y1 = table.get(valueBelow);
        Double y2 = table.get(valueAbove);
        
        return ((y2 - y1) / (x2 - x1)) * (key - x1) + y1

    }
}