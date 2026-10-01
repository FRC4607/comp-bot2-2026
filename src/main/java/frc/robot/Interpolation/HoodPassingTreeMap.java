package frc.robot.Interpolation;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class HoodPassingTreeMap extends InterpolatingTreeMap<Double, Double> {

    public HoodPassingTreeMap() {
        super(InverseInterpolator.forDouble(), HoodPassingTreeMap::interpolateValues);
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
     * @return A populated HoodPassingTreeMap.
     */
    public static HoodPassingTreeMap createDefaultMap() {
        HoodPassingTreeMap map = new HoodPassingTreeMap();
        map.put(1.0, 2.25);
        map.put(2.0, 2.25);
        map.put(4.5, 1.3);
        map.put(6.0, 2.25);
        map.put(8.5, 2.25);
        map.put(14.0, 2.25);
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