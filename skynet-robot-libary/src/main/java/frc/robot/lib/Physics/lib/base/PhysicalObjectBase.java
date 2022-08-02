package frc.robot.lib.Physics.lib.base;

import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

import frc.robot.lib.Physics.Constants;

public abstract class PhysicalObjectBase {
    protected State state;
    protected Scalar estemation_resulotion;
    protected Scalar mass;
    protected Scalar moment_of_inertia;
    protected int kinematic_resulotion;

    public void add_force(Vector force) {
        state.sigma_forces.add(force);
    }

    public void subtract_force(Vector force) {
        state.sigma_forces.subtract(force);
    }
    public static int factorial(int number)
    {
        int sum = 1;
        for (int i=1; i<=number; i++)
        {
            sum *=i;
        }
        return sum;
    }

    public void calc() {
        state.acceleration = state.sigma_forces.divide(mass);
        state.liniar_moment = state.velocity.multiply(mass);
        state.potential_energy = mass
                .multiply(Constants.gravitational_acceleration.getMagnitude()).multiply(state.position.getComponents().get(2));
        state.potential_energy = mass
                .multiply(state.position.getComponents().get(2) * Constants.gravitational_acceleration.getMagnitude());
        state.energy = state.potential_energy
                .add(mass.multiply(state.velocity.getMagnitude() * state.velocity.getMagnitude() * 0.5))
                .add(moment_of_inertia.multiply(
                        state.rotational_velocity.getMagnitude() * state.rotational_velocity.getMagnitude() * 0.5));
        state.angular_momentum = state.rotational_velocity.multiply(moment_of_inertia);
    }

    protected void recursive_update(State before_state, int turn) {
        if (turn == state.kinematics_varuibales.size()) {
            return;
        }
        if (turn == 1) {
            Vector before_value = before_state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn - 1);
            Vector current_value = state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn - 1);
            state.kinematics_varuibales.set(state.kinematics_varuibales.size() - turn,
                    current_value.subtract(before_value).divide(estemation_resulotion));
        } else {
            for (int i = 1; turn > i; i++) {
                Vector before_vector = before_state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn - i);
                Vector rate_of_change = state.kinematics_varuibales.get(state.kinematics_varuibales.size() - i);
                state.kinematics_varuibales.set(state.kinematics_varuibales.size() - turn - i, before_vector.add(rate_of_change.divide(factorial(i)).multiply(Math.pow(estemation_resulotion.getMagnitude(),i))));

            }
        }
        recursive_update(before_state, turn+1);
    }

    public void update_position() {
        State before_state = new State(state);
        calc();
        state.save_to_list();
        recursive_update(before_state, 1);
    }

    public static void main(String[] args) {
        System.out.println(factorial(12));

    }
}
