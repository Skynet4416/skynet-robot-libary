package frc.robot.lib.Physics;

import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

public final class Constants {
    public static final Vector gravitational_acceleration = new Vector(0.0, -9.807, 0.0); // m/s^2
    public static final Scalar density_of_air = () -> 1.23; // kg/m^3
}
