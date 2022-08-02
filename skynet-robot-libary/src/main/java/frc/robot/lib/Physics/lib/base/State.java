package frc.robot.lib.Physics.lib.base;

import java.util.ArrayList;
import java.util.List;

import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

public class State {
    public Vector position; // meters
    public Vector velocity; // m/s
    public Vector acceleration; // m/s^1
    public Vector sigma_forces; // N, kg*m/s^2
    public Vector rotational_velocity; // radians/s
    public Vector rotational_acceleration; // radians / s^2
    public Scalar energy; // j, kg*m^2/2
    public Scalar potential_energy; // j, kg*m^2/2
    public Vector liniar_moment; // kg*m/s, n*s
    public Vector angular_momentum; // kg*m^2/s
    public Vector jerk; //m/s^3
    public List<Vector> kinematics_varuibales;

    public State() {
        this.position = new Vector(0.0,0.0,0.0);
        this.velocity = new Vector(0.0,0.0,0.0);
        this.acceleration = new Vector(0.0,0.0,0.0);
        this.sigma_forces = new Vector(0.0,0.0,0.0);
        this.rotational_velocity = new Vector(0.0,0.0,0.0);
        this.rotational_acceleration = new Vector(0.0,0.0,0.0);
        this.liniar_moment = new Vector(0.0,0.0,0.0);
        this.angular_momentum = new Vector(0.0,0.0,0.0);
        this.energy = () -> 0.0;
        this.potential_energy = () -> 0.0;
        this.jerk = new Vector(0.0,0.0,0.0);
        this.kinematics_varuibales = new ArrayList<Vector>();
    }
    public void save_to_list()
    {
        this.kinematics_varuibales.add(0, position);
        this.kinematics_varuibales.add(1, velocity);
        this.kinematics_varuibales.add(2, acceleration);
        this.kinematics_varuibales.add(3, jerk);

    }
    public void save_to_var()
    {
        this.position = kinematics_varuibales.get(0);
        this.velocity = kinematics_varuibales.get(1);
        this.acceleration = kinematics_varuibales.get(2);
        this.jerk = kinematics_varuibales.get(3);
    }
    public State(State state)
    {
        this.position = state.position;
        this.velocity =state.velocity;
        this.acceleration = state.acceleration;
        this.sigma_forces =state.sigma_forces;
        this.rotational_velocity = state.rotational_velocity;
        this.rotational_acceleration = state.rotational_acceleration;
        this.liniar_moment = state.liniar_moment;
        this.angular_momentum = state.angular_momentum;
        this.energy = state.energy;
        this.potential_energy = state.potential_energy;
        this.jerk = state.jerk;
    }
}
