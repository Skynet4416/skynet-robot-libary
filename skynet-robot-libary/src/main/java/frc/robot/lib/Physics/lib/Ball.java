package frc.robot.lib.Physics.lib;

import java.util.ArrayList;
import java.util.Arrays;
import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

import frc.robot.lib.Physics.lib.base.PhysicalObjectBase;
import frc.robot.lib.Physics.lib.base.State;
import frc.robot.lib.Meth.Target;
import frc.robot.lib.Meth.shooter_optimiztion;
import frc.robot.lib.Meth.shooter_optimiztion.OptimizationType;
import frc.robot.lib.Physics.Constants;

public class Ball extends PhysicalObjectBase {
    protected Scalar radius;
    protected Scalar drag_constant;
    protected Scalar magnus_constant;
    protected Scalar cross_section_area_of_ball;
    protected Scalar lift_coeficent;

    public Ball(double mass, double radius, double drag_constant, double estemation_resulotion) {
        this.estemation_resulotion = () -> estemation_resulotion;
        this.mass = () -> mass;
        state = new State();
        this.drag_constant = () -> drag_constant;
        this.radius = () -> radius;
        cross_section_area_of_ball = this.radius.multiply(this.radius).multiply(Math.PI);
        this.moment_of_inertia = () -> mass * radius * radius;
    }

    public State get_state() {
        return this.state;
    }

    public void set_state(State state) {
        this.state = state;
    }

    public double get_radius() {
        return radius.getMagnitude();
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

    public void calc_magnus_forces() {
        state.sigma_forces = new Vector(0.0, 0.0, 0.0);
        Vector velocity_direction = state.velocity.divide(state.velocity.getMagnitude());
        add_force(Constants.gravitational_acceleration.multiply(mass)); // gravity
        add_force(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
                .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(), 2))); // drag =
                                                                                                             // -1/2 *
                                                                                                             // Cd * p *
                                                                                                             // A * v^2
                                                                                                             // *v(vector)/abs(v)
        // System.out.println(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
        // .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(),
        // 2)));
        if (Math.abs(state.rotational_velocity.getMagnitude()) > 0) {
            Vector axis_of_rotation = state.rotational_velocity.divide(state.rotational_velocity.getMagnitude());

            add_force(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
                    .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
                    .multiply(Math.pow(state.velocity.getMagnitude(), 2))); // magnus = 1/2 cl * p * A * v^2 *
                                                                            // (v(vector)/abd(v) X
                                                                            // rotational_velocity(vector)/abs(rotational_velocity))
            // System.out.println(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
            // .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
            // .multiply(Math.pow(state.velocity.getMagnitude(), 2)));
        }

    }

    public void calc_magnus_forces_nasa() {
        state.sigma_forces = new Vector(0.0, 0.0, 0.0);
        Vector velocity_direction = state.velocity.divide(state.velocity.getMagnitude());
        add_force(Constants.gravitational_acceleration.multiply(mass)); // gravity
        add_force(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
                .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(), 2))); // drag =
                                                                                                             // -1/2 *
                                                                                                             // Cd * p *
                                                                                                             // A * v^2
                                                                                                             // *v(vector)/abs(v)
        // System.out.println(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
        // .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(),
        // 2)));
        Vector axis_of_rotation = state.rotational_velocity.divide(state.rotational_velocity.getMagnitude());
        System.out.println(velocity_direction.crossProduct3D(axis_of_rotation)
                .multiply(5.33333333 * Math.PI * Math.PI * Math.pow(radius.getMagnitude(), 3)
                        * state.rotational_velocity.getMagnitude() * Constants.density_of_air.getMagnitude()
                        * state.velocity.getMagnitude()));

        add_force(velocity_direction.crossProduct3D(axis_of_rotation)
                .multiply(5.33333333 * Math.PI * Math.PI * Math.pow(radius.getMagnitude(), 3)
                        * state.rotational_velocity.getMagnitude() * Constants.density_of_air.getMagnitude()
                        * state.velocity.getMagnitude())); // magnus = 1/2 cl * p * A * v^2 *
        // (v(vector)/abd(v) X
        // rotational_velocity(vector)/abs(rotational_velocity))
        // System.out.println(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
        // .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
        // .multiply(Math.pow(state.velocity.getMagnitude(), 2)));
    }

    public void calc_no_magnusforces() {
        state.sigma_forces = new Vector(0.0, 0.0, 0.0);
        Vector velocity_direction = state.velocity.divide(state.velocity.getMagnitude());
        add_force(Constants.gravitational_acceleration.multiply(mass)); // gravity
        add_force(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
                .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(), 2))); // drag =
                                                                                                             // -1/2 *
                                                                                                             // Cd * p *
                                                                                                             // A * v^2
                                                                                                             // *v(vector)/abs(v)
        // System.out.println(velocity_direction.multiply(-0.5).multiply(drag_constant).multiply(Constants.density_of_air)
        // .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(),
        // 2)));
        Vector axis_of_rotation = state.rotational_velocity.divide(state.rotational_velocity.getMagnitude());

        // add_force(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
        // .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
        // .multiply(Math.pow(state.velocity.getMagnitude(), 2))); // magnus = 1/2 cl *
        // p * A * v^2 *
        // (v(vector)/abd(v) X
        // rotational_velocity(vector)/abs(rotational_velocity))
        // System.out.println(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
        // .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
        // .multiply(Math.pow(state.velocity.getMagnitude(), 2)));
    }

    public ArrayList<State> simulate_ball(Boolean print) {
        this.before_before_state = new State(this.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        lift_coeficent = radius.multiply(state.rotational_velocity.getMagnitude())
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

    public static void test_magnus_recursion() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, 0.0));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        pos_array.add(2, new ArrayList<Double>());
        pos_array.add(3, new ArrayList<Double>());
        ArrayList<State> states_array = new ArrayList<State>();
        states_array.add(new State(ball.state));
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.calc_magnus_forces();
            ball.update_position(i);
            pos_array.get(0).add(ball.state.position.getComponent(0)); // x
            pos_array.get(1).add(ball.state.position.getComponent(1)); // y
            pos_array.get(2).add(ball.state.position.getComponent(2)); // z
            pos_array.get(3).add(ball.state.velocity.getDegree()); // alpha
            states_array.add(new State(ball.state));
            i++;
        }
        // System.out.print("\n\n\nx_array = ");
        // System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        // System.out.print("\n\n\ny_array = ");
        // System.out.println(Arrays.deepToString(pos_array.get(1).toArray()));

    }

    public static void test_drag_recursion() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.calc_no_magnusforces();
            ball.update_position(i);
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
            i++;
        }
        // System.out.print("\n\n\nx_array_drag_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        // System.out.print("\n\n\ny_array_drag_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void test_drag_no_recursion() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.calc_no_magnusforces();
            ball.update_position_no_recursion();
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
            i++;
        }
        // System.out.print("\n\n\nx_array_drag_no_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        // System.out.print("\n\n\ny_array_drag_no_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void test_lift_no_recursion() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));
        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.calc_magnus_forces();
            ball.update_position_no_recursion();
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
            i++;
        }
        // System.out.print("\n\n\nx_array_lift_no_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        // System.out.print("\n\n\ny_array_lift_no_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void test_more_parameters() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        for (i = 0; i < 2; i++) {
            ball.calc_magnus_forces();
            ball.update_position(i);
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
        }
        // System.out.print("\n\n\nx_array_lift_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        // System.out.print("\n\n\ny_array_lift_recursion = ");
        // System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void test_magnus_adjusted_lift_recursion() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                    .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl =
                                                                 // R*rotational_velocity/v
                                                                 // which/ is S
            ball.calc_magnus_forces();
            ball.update_position(i);
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
            i++;
        }
        System.out.print("\n\n\nx_array_changing_lift_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        System.out.print("\n\n\ny_array_changing_lift_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void test_magnus_recursion_nasa() {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.calc_magnus_forces_nasa();
            ball.update_position(i);
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
            i++;
        }
        System.out.print("\n\n\nx_array_nasa_lift_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        System.out.print("\n\n\ny_array_nasa_lift_recursion = ");
        System.out.println(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void main(String[] args) {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.state.kinematics_varuibales.add(new Vector(0.0, 0.0, 0.0));

        Target hub = new Target(new Vector(16.0, 2.7178, 1.0), new Vector(1.22, 0.5,
                1.0), 999, -999, 999, -999);
        shooter_optimiztion.optimize(ball, hub,
                OptimizationType.MINIMIZE, OptimizationType.MINIMIZE,
                OptimizationType.IGNORE,
                OptimizationType.IGNORE, OptimizationType.IGNORE, OptimizationType.IGNORE,
                90.0, 45.0, 5000.0, 0.0);

        // ball.set_position(new Vector(0.0, 0.1, 0.0));
        // ball.set_started_velocity(new Vector(15.688027284833968, 17.37157334954557,
        // 0.0));
        // ball.set_rotational_velocity(new Vector(0.0, 0.0, 0.0));
        // ball.simulate_ball(true);

    }
}
