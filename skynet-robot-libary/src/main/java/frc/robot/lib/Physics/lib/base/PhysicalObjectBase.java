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
    public State before_before_state;

    public void add_force(Vector force) {
        state.sigma_forces = state.sigma_forces.add(force);
    }

    public void subtract_force(Vector force) {
        state.sigma_forces = state.sigma_forces.subtract(force);
    }

    public static int factorial(int number) {
        int sum = 1;
        for (int i = 1; i <= number; i++) {
            sum *= i;
        }
        return sum;
    }

    public void add_torque(Vector force) {

    }

    public void calc() {
        state.acceleration = state.sigma_forces.divide(mass);
        state.rotational_acceleration = state.torque.divide(moment_of_inertia);

        state.liniar_moment = state.velocity.multiply(mass);
        state.potential_energy = mass
                .multiply(Constants.gravitational_acceleration.getMagnitude())
                .multiply(state.position.getComponents().get(1));
        state.energy = state.potential_energy
                .add(mass.multiply(state.velocity.getMagnitude() * state.velocity.getMagnitude() * 0.5))
                .add(moment_of_inertia.multiply(
                        state.rotational_velocity.getMagnitude() * state.rotational_velocity.getMagnitude() * 0.5));
        state.angular_momentum = state.rotational_velocity.multiply(moment_of_inertia);
    }

    protected void recursive_update(State before_state, int turn, int iteration) {
        // System.out.println("Turn: " + turn);
        if (turn > state.kinematics_varuibales.size()) {
            return;
        } else if (state.kinematics_varuibales.size() - turn == 2) {
            // System.out.println("trying to set acceleration");
        } else if (state.kinematics_varuibales.size() - turn > 2) {
            Vector before_acceleration = before_state.acceleration;
            Vector current_acceleration = state.acceleration;
            state.kinematics_varuibales.set(state.kinematics_varuibales.size() - turn,
                    current_acceleration.subtract(before_acceleration).divide(Math
                            .pow(estemation_resulotion.getMagnitude(), state.kinematics_varuibales.size() - turn - 2)));
        } else {
            Vector current_vector = state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn);
            // System.out.println("current vector place " +
            // (state.kinematics_varuibales.size() - turn));
            // System.out.println("loop");
            for (int i = 1; turn > i; i++) {
                if (state.kinematics_varuibales.size() - turn + i > 2) {

                }
                // System.out.println("adding vector place" +
                // (state.kinematics_varuibales.size() - turn + i));
                Vector adding_vector = state.kinematics_varuibales
                        .get(state.kinematics_varuibales.size() - turn + i);
                // System.out.println(current_vector.getMagnitude() + " += " +
                // adding_vector.getMagnitude() + " * "
                // + estemation_resulotion.getMagnitude() + "^" + i + "/" + factorial(i));
                current_vector = current_vector.add(
                        adding_vector.multiply(Math.pow(estemation_resulotion.getMagnitude(), i)).divide(factorial(i)));

            }
            state.kinematics_varuibales.set(state.kinematics_varuibales.size() - turn, current_vector);
            // System.out.println("end loop");

        }
        recursive_update(before_state, turn + 1, iteration);
    }

    protected void recursive_update1(State before_state, int turn, int iteration) {
        System.out.println("Turn: " + turn);
        if (turn > state.kinematics_varuibales.size()) {
            return;
        } else if (state.kinematics_varuibales.size() - turn == 2) {
            System.out.println("trying to set acceleration");
        } else if (turn == 1) {
            System.out.println("first value");
            Vector before_value = before_state.kinematics_varuibales
                    .get(state.kinematics_varuibales.size() - turn - 1);
            Vector current_value = state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn - 1);
            state.kinematics_varuibales.set(state.kinematics_varuibales.size() - turn,
                    current_value.subtract(before_value).divide(estemation_resulotion));
            System.out.println(state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn));
        } else {
            Vector current_vector = state.kinematics_varuibales.get(state.kinematics_varuibales.size() - turn);
            System.out.println("current vector place " + (state.kinematics_varuibales.size() - turn));
            System.out.println("loop");
            for (int i = 1; turn > i; i++) {
                if (state.kinematics_varuibales.size() - turn + i > 2) {

                }
                System.out.println("adding vector place" + (state.kinematics_varuibales.size() - turn + i));
                Vector adding_vector = state.kinematics_varuibales
                        .get(state.kinematics_varuibales.size() - turn + i);
                System.out.println(current_vector.getMagnitude() + " += " + adding_vector.getMagnitude() + " * "
                        + estemation_resulotion.getMagnitude() + "^" + i + "/" + factorial(i));
                current_vector = current_vector.add(
                        adding_vector.multiply(Math.pow(estemation_resulotion.getMagnitude(), i)).divide(factorial(i)));

            }
            state.kinematics_varuibales.set(state.kinematics_varuibales.size() - turn, current_vector);
            System.out.println("end loop");

        }
        recursive_update(before_state, turn + 1, iteration);
    }

    public void update_position_no_recursion() {
        State before_state = new State(state);
        calc();
        state.velocity = before_state.velocity.add(state.acceleration.multiply(estemation_resulotion));
        state.position = before_state.position.add(state.velocity.multiply(estemation_resulotion))
                .add(state.acceleration.multiply(0.5).multiply(estemation_resulotion).multiply(estemation_resulotion));
        // System.out.println("\nFORCES: " + state.sigma_forces);

    }

    public void update_position(int iteration) {
        // System.out.println("ITERATION " + iteration);
        State before_state = new State(state);
        calc();
        state.save_to_list();
        recursive_update(before_state, 1, iteration);
        state.save_to_var();
        this.before_before_state = new State(before_state);

    }

}
