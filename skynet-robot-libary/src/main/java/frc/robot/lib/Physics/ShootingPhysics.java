package frc.robot.lib.Physics;

public class ShootingPhysics {
    public static double get_starting_velocity_for_distance_and_angle(double angle, double direct_distance)
    {
        double radians = Math.toRadians(angle);
        return Math.sqrt((Constants.gravitational_acceleration*direct_distance*Math.cos(radians)) / ((2* Math.sin(radians) * (1-Math.cos(radians)))));
    }
}
