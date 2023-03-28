package frc.robot.lib.Physics.lib;

import java.util.ArrayList;
import java.util.Arrays;

import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

import edu.wpi.first.math.util.Units;
import frc.robot.lib.Meth.Target;
import frc.robot.lib.Meth.shooter_optimiztion;
import frc.robot.lib.Physics.Constants;
import frc.robot.lib.Physics.lib.base.PhysicalObjectBase;
import frc.robot.lib.Physics.lib.base.State;;

public class Cube extends PhysicalObjectBase {
    protected Scalar length;
    protected Scalar width;
    protected Scalar drag_constant;
    protected Scalar magnus_constant;
    protected Scalar cross_section_area_of_cube;
    protected Scalar lift_coeficent;

    public State get_state() {
        return this.state;
    }

    public void set_state(State state) {
        this.state = state;
    }

    public double get_radius() {
        return length.getMagnitude();
    }

    public double get_target_threshold() {
        Double tolerance = 0.1; // IN METERS
        return length.getMagnitude() * 4 + tolerance;
    }

    public void set_position(Vector position) {
        state.position = position;
    }

    public void set_started_velocity(Vector velocity) {
        state.velocity = velocity;
    }

    public void set_rotational_velocity(Vector rotational_velocity) {
        state.rotational_velocity = rotational_velocity;
    }

    public Vector get_position() {
        return state.position;
    }

    public Cube(double mass, double length, double width, double drag_constant, double lift_coeficent,
            double estemation_resulotion) {
        this.estemation_resulotion = () -> estemation_resulotion;
        this.mass = () -> mass;
        state = new State();
        this.drag_constant = () -> drag_constant;
        this.length = () -> length;
        this.width = () -> width;
        cross_section_area_of_cube = this.length.multiply(this.width);
        this.moment_of_inertia = () -> mass * Math.pow(length * width, 2) / 12;
        this.lift_coeficent = () -> lift_coeficent;
    }

    public void calc_magnus_forces() {
        state.sigma_forces = new Vector(0.0, 0.0, 0.0);
        Vector velocity_direction = state.velocity.divide(state.velocity.getMagnitude());
        add_force(Constants.gravitational_acceleration.multiply(mass)); // gravity
        add_force(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
                .multiply(cross_section_area_of_cube).multiply(Math.pow(state.velocity.getMagnitude(), 2))); // drag =
                                                                                                             // -1/2 *
                                                                                                             // Cd * p *
                                                                                                             // A * v^2
                                                                                                             // *v(vector)/abs(v)
        // System.out.println(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
        // .multiply(cross_section_area_of_cube).multiply(Math.pow(state.velocity.getMagnitude(),
        // 2)));
        if (Math.abs(state.rotational_velocity.getMagnitude()) > 0) {
            Vector axis_of_rotation = state.rotational_velocity.divide(state.rotational_velocity.getMagnitude());

            add_force(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
                    .multiply(Constants.density_of_air).multiply(cross_section_area_of_cube)
                    .multiply(Math.pow(state.velocity.getMagnitude(), 2))); // magnus = 1/2 cl * p * A * v^2 *
                                                                            // (v(vector)/abd(v) X
                                                                            // rotational_velocity(vector)/abs(rotational_velocity))
            // System.out.println(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
            // .multiply(Constants.density_of_air).multiply(cross_section_area_of_cube)
            // .multiply(Math.pow(state.velocity.getMagnitude(), 2)));
        }

    }

    public ArrayList<State> simulate_object_runge_kutta(boolean print) {
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());

        ArrayList<State> states_array = new ArrayList<State>();
        states_array.add(new State(this.state));
        this.before_before_state = new State(this.state);

        while (this.state.position.getComponent(1) > 0) {
            State before_state = new State(this.state);

            this.calc_magnus_forces();
            this.calc();
            this.runge_kutta_aproxemation(before_state);
            pos_array.get(0).add(this.state.position.getComponent(0));
            pos_array.get(1).add(this.state.position.getComponent(1));
            this.before_before_state = new State(before_state);

            states_array.add(new State(this.state));
        }
        if (print) {
            System.out.print("\n\n\nx_array_runge_kutta = ");
            System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
            System.out.print("\n\n\ny_array_runge_kutta  = ");
            System.out.println(Arrays.deepToString(pos_array.get(1).toArray()));
        }
        return states_array;

    }

    public static void simulte_from_rpm_and_angle_runge_kutta(Cube projectile, double TopRPM, double BottomRPM,
            double angle) {
        double TopinDiameter = 4;
        double TopmDiameter = Units.inchesToMeters(TopinDiameter);
        double TopCircumference = (TopmDiameter) * Math.PI;

        double BottominDiameter = 4;
        double BottommDiameter = Units.inchesToMeters(BottominDiameter);
        double BottomCircumference = (BottommDiameter) * Math.PI;
        double TopRPS = TopRPM / 60.0;
        double BottomRPS = BottomRPM / 60.0;
        double muzzle_velocity = (TopRPS * TopCircumference + BottomRPS * BottomCircumference) / 2.0;
        double angle_rads = Math.toRadians(angle); // IN RADIANS

        double TopRads = 0.104719755 * TopRPM;
        double BottompRads = 0.104719755 * BottomRPM;

        Vector started_velocity = new Vector(muzzle_velocity * Math.sin(angle_rads),
                muzzle_velocity * Math.cos(angle_rads), 0.0); // m/s
        Vector started_rotational_velocity = new Vector(0.0,
                (TopRads * TopmDiameter / 2 - BottompRads * BottommDiameter / 2)
                        / (2 * projectile.get_radius()),
                0.0);
        ; // radians

        projectile.set_position(new Vector(0.0, 0.86, 0.0));
        projectile.set_started_velocity(started_velocity);
        projectile.set_rotational_velocity(started_rotational_velocity);

        // ball.lift_coeficent =
        // ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
        // .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl
        // = R*rotational_velocity/v
        // which/ is S
        projectile.simulate_object_runge_kutta(true);

    }

    public static void simulte_from_rpm_and_angle(Cube projectile, double TopRPM, double BottomRPM, double angle) {
        double TopinDiameter = 4;
        double TopmDiameter = Units.inchesToMeters(TopinDiameter);
        double TopCircumference = (TopmDiameter) * Math.PI;

        double BottominDiameter = 4;
        double BottommDiameter = Units.inchesToMeters(BottominDiameter);
        double BottomCircumference = (BottommDiameter) * Math.PI;
        double TopRPS = TopRPM / 60.0;
        double BottomRPS = BottomRPM / 60.0;
        double muzzle_velocity = (TopRPS * TopCircumference + BottomRPS * BottomCircumference) / 2.0;
        double angle_rads = Math.toRadians(angle); // IN RADIANS

        double TopRads = 0.104719755 * TopRPM;
        double BottompRads = 0.104719755 * BottomRPM;

        Vector started_velocity = new Vector(muzzle_velocity * Math.sin(angle_rads),
                muzzle_velocity * Math.cos(angle_rads), 0.0); // m/s
        Vector started_rotational_velocity = new Vector(0.0, 0.0,
                (TopRads * TopmDiameter / 2 - BottompRads * BottommDiameter / 2)
                        / (2 * projectile.get_radius())); // radians

        projectile.set_position(new Vector(0.0, 0.1, 0.0));
        projectile.set_started_velocity(started_velocity);
        projectile.set_rotational_velocity(started_rotational_velocity);
        projectile.simulate_object(true);

    }

    @Override
    public ArrayList<State> simulate_object(Boolean print) {
        this.before_before_state = new State(this.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        lift_coeficent = length.multiply(state.rotational_velocity.getMagnitude())
                .divide(state.velocity.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        pos_array.add(2, new ArrayList<Double>());
        pos_array.add(3, new ArrayList<Double>());
        ArrayList<State> states_array = new ArrayList<State>();
        states_array.add(new State(this.state));
        int i = 0;
        while (this.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            this.calc_magnus_forces();
            this.update_position(i);
            pos_array.get(0).add(this.state.position.getComponent(0)); // x
            pos_array.get(1).add(this.state.position.getComponent(1)); // y
            pos_array.get(2).add(this.state.position.getComponent(2)); // z
            pos_array.get(3).add(this.state.velocity.getDegree()); // alpha
            states_array.add(new State(this.state));
            i++;
        }

        if (print) {
            System.out.print("\n\n\nx_array = ");
            System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
            System.out.print("\n\n\ny_array = ");
            System.out.println(Arrays.deepToString(pos_array.get(1).toArray()));
        }

        return states_array;
    }

    public static void main(String[] args) {
        Cube cube = new Cube(0.071, 0.24, 0.24, 1.2, 0.2, 0.02);
        cube.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        Target hub = new Target(new Vector(1.0, 0.826, 0.0), new Vector(1.22 -
                cube.get_target_threshold(), 0.05,
                1.0), 90, 45, 999, -999);

        // // ANGLE FROM Y AXIS ^
        Vector results;
        // Vector results = shooter_optimiztion.binary_smart_optimize(cube, hub, 45.0,
        // 0.0, 5000.0, 1500.0, 12.0);

        // simulte_from_rpm_and_angle(cube, results.getComponent(0),
        // results.getComponent(1), results.getComponent(2));

        results = shooter_optimiztion.binary_smart_optimize_runge_kutta(cube, hub, 45.0,
                0.0, 5000.0, 1500.0, 12.0);

        simulte_from_rpm_and_angle_runge_kutta(cube, results.getComponent(0),
                results.getComponent(1), results.getComponent(2));
    }

}
