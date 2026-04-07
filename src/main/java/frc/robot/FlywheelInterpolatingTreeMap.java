package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class FlywheelInterpolatingTreeMap extends InterpolatingTreeMap<Double, Double> {

    public FlywheelInterpolatingTreeMap() {
        super(InverseInterpolator.forDouble(), FlywheelInterpolatingTreeMap::interpolateValues);
    }

    /**
     * Interpolation function for the tree map.
     * Linearly interpolates between two values based on the proportion t.
     *
     * @param start The starting value.
     * @param end The ending value.
     * @param t The proportion between 0.0 and 1.0.
     * @return The interpolated value.
     */
    public static Double interpolateValues(Double start, Double end, double t) {
        return start + (end - start) * t;
    }

    /**
     * Factory method to create and populate the tree map with default values.
     *
     * @return A populated GenericInterpolatingTreeMap.
     */
    public static FlywheelInterpolatingTreeMap createDefaultMap() {
        FlywheelInterpolatingTreeMap map = new FlywheelInterpolatingTreeMap();
        map.put(1.016, 55.0);
        map.put(1.672, 45.0);
        map.put(3.501, 50.0);
        map.put(4.466, 60.0);
        map.put(5.72, 65.0);
        return map;
    }

    /**
     * Interpolates a value based on the given key.
     *
     * @param key The key to interpolate.
     * @return The interpolated value, or null if the key is out of range.
     */
    public Double interpolate(double key) {
        return this.get(key);
    }
}