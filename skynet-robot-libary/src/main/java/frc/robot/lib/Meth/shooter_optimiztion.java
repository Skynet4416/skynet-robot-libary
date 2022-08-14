package frc.robot.lib.Meth;

import com.github.iprodigy.physics.util.vector.Vector;

import frc.robot.lib.Physics.lib.Ball;

public final class shooter_optimiztion {
    public enum OptimizationType {
        MAXIMIZE,
        MINIMIZE,
        IGNORE
    }

    private final Integer failure_tolerance = 3;

    // Target hub = new Target(new Vector(16.0, 2.7178, 1.0), new Vector(1.22, 0.5,
    // 1.0), 999, -999, 999, -999);
    public static void optimize(Ball projectile,Target target, OptimizationType shooting_angle, OptimizationType shooting_RPM,
            OptimizationType entrance_velocity,
            OptimizationType entrance_angle, OptimizationType trajectory_length) {

        double TopinDiameter = 4;
        double TopmDiameter = (TopinDiameter * 25.4) / 1000;
        double TopCircumference = (TopmDiameter) * Math.PI;

        double BottominDiameter = 4;
        double BottommDiameter = (TopinDiameter * 25.4) / 1000;
        double BottomCircumference = (TopmDiameter) * Math.PI;

        double TopRPM = 5000;
        double TopRPS = TopRPM / 60.0;
        double BottomRPM = 5000;
        double BottomRPS = BottomRPM / 60.0;

        double muzzle_velocity = (TopRPS * TopCircumference + BottomRPS * BottomCircumference) / 2.0; // SURFACE SPEED
        double angle_rads = 0; // IN RADIANS

        double TopRads = 0.104719755 * TopRPM;
        double BottompRads = 0.104719755 * BottomRPM;

        Vector started_velocity = new Vector(muzzle_velocity * Math.sin(angle_rads),
                muzzle_velocity * Math.cos(angle_rads)); // m/s
        Vector started_rotational_velocity = new Vector(0.0,0.0,(TopRads * TopmDiameter /2  - BottompRads * BottommDiameter /2) / (2*projectile.get_radius())); // radians

        projectile.set_started_velocity(started_velocity);
        projectile.set_rotational_velocity(started_rotational_velocity);

        
    }
}


