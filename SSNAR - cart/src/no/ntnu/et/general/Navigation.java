/**
 * This code is written as a part of a Master Thesis
 * the fall of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.general;

/**
 *
 * @author geirhei
 */
public class Navigation {
    
    /**
     * Method for retrieving the index of the array that has the lowest value
     * 
     * @param distances array [0,360)
     * @return int angle [0,360)
     */
    public static int getShortestDistanceHeading(int[] distances) {
        int angle = -1;
        int shortest = Integer.MAX_VALUE;
        for (int i = 0; i < distances.length; i++) {
            if (distances[i] < shortest) {
                angle = i;
            }
        }
        return angle;
    }
    
    // Was not originaly in this project, but estimatet it is from the same author
    /**
     * Calculates and fills an int[4] array with the measured distances in all four directions around the robot.
     * In the order [d_front, d_left, d_back, d_right]
     * Only updates values when servo step is below 30 and above 60. Else the distances are unchanged,
     * or set to max line of sight if == 0
     * 
     * @param measurements double[]
     * @param distances int[] containing the distances in the respective distances around the robot.
     * @param servoStep the number of steps the servo tower is rotated
     * @return updated distance array of integers
     */
    public static int[] calculateDistances(double[] measurements, int[] distances, int servoStep) {
        if (measurements.length != 4) {
            System.out.println("Invalid length of []measuremenets.");
            return null;
        }
        if (servoStep < 0 || servoStep > 90) {
            System.out.println("Servo step out of range.");
            return null;
        }
        
        double servoStepRad = Math.toRadians(servoStep);
        if (servoStep >= 0 && servoStep <= 30) {
            for (int i = 0; i < measurements.length; i++) {
                distances[i] = (int) (measurements[i] * Math.cos(servoStepRad));
            }
        } else if (servoStep >= 60 && servoStep <= 90) {
            distances[0] = (int) (measurements[3] * Math.sin(servoStepRad));
            for (int i = 1; i < measurements.length; i++) {
                distances[i] = (int) (measurements[i-1] * Math.sin(servoStepRad));
            }
        }
        for (int i = 0; i < distances.length; i++) {
            if (distances[i] == 0 || distances[i] > 40) {
                distances[i] = 40;
            }
        }
        return distances;
    }
    
    /// Author Karoline Halvorsen. FInd the distance from th IR sensors
    /**
     * Calculates and fills an int[4] array with the measured distances in all four directions around the robot.
     * In the order [d_front, d_left, d_back, d_right]
     * Only updates values when servo step is below 30 and above 60. Else the distances are unchanged,
     * or set to max line of sight if == 0
     * 
     * @param measurements double[]
     * @param distances int[] containing the distances in the respective distances around the robot.
     * @param servoStep the number of steps the servo tower is rotated
     * @return updated distance array of integers
     */
    public static int[] calculateDistances2(double[] measurements, int[] distances, int servoStep) {
        if (measurements.length != 4) {
            System.out.println("Invalid length of []measuremenets.");
            return null;
        }
        if (servoStep < 0 || servoStep > 90) {
            System.out.println("Servo step out of range.");
            return null;
        }
        
        double servoStepRad = Math.toRadians(servoStep);
        if (servoStep >= 0 && servoStep <= 30) {
            for (int i = 0; i < measurements.length; i++) {
                distances[i] = (int) (measurements[i] * Math.cos(servoStepRad));
            }
        } else if (servoStep >= 60 && servoStep <= 90) {
            distances[0] = (int) (measurements[3] * Math.sin(servoStepRad));
            for (int i = 1; i < measurements.length; i++) {
                distances[i] = (int) (measurements[i-1] * Math.sin(servoStepRad));
            }
        }
        for (int i = 0; i < distances.length; i++) {
            if (distances[i] == 0 || distances[i] > 40) {
                distances[i] = 40;
            }
        }
        return distances;
    }
    
    
}
