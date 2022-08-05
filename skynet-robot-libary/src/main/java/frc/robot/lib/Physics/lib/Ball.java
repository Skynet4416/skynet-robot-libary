package frc.robot.lib.Physics.lib;

import java.util.ArrayList;
import java.util.Arrays;
import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

import frc.robot.lib.Physics.lib.base.PhysicalObjectBase;
import frc.robot.lib.Physics.lib.base.State;
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
        //         .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
        //         .multiply(Math.pow(state.velocity.getMagnitude(), 2))); // magnus = 1/2 cl * p * A * v^2 *
                                                                        // (v(vector)/abd(v) X
                                                                        // rotational_velocity(vector)/abs(rotational_velocity))
        // System.out.println(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
        // .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
        // .multiply(Math.pow(state.velocity.getMagnitude(), 2)));
    }
    public static void test_magnus_recursion()
    {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
        ball.before_before_state = new State(ball.state);
        // System.out.println(ball.cross_section_area_of_ball.getMagnitude());
        ArrayList<ArrayList<Double>> pos_array = new ArrayList<ArrayList<Double>>();
        pos_array.add(0, new ArrayList<Double>());
        pos_array.add(1, new ArrayList<Double>());
        int i = 0;
        while (ball.state.position.getComponent(1) > 0) {
            // System.out.println(i + " iteration");
            ball.calc_magnus_forces();
            ball.update_position(i);
            pos_array.get(0).add(ball.state.position.getComponent(0));
            pos_array.get(1).add(ball.state.position.getComponent(1));
            i++;
        }
        System.out.print("\n\n\nx_array_lift_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        System.out.print("\n\n\ny_array_lift_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }
    public static void test_drag_recursion()
    {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
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
        System.out.print("\n\n\nx_array_drag_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        System.out.print("\n\n\ny_array_drag_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }
    public static void test_drag_no_recursion()
    {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
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
        System.out.print("\n\n\nx_array_drag_no_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        System.out.print("\n\n\ny_array_drag_no_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }
    public static void test_lift_no_recursion()
    {
        Ball ball = new Ball(0.26932047, 0.12065, 0.47, 0.1);
        ball.set_started_velocity(new Vector(19.9366967045, 19.9366967045, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -58.422600157894736842105263157895));
        ball.lift_coeficent = ball.radius.multiply(ball.state.rotational_velocity.getMagnitude())
                .divide(ball.state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v
                                                             // which/ is S
        ball.set_position(new Vector(0.0, 0.0000001, 0.0));
        ball.state.save_to_list();
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
        System.out.print("\n\n\nx_array_lift_no_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(0).toArray()));
        System.out.print("\n\n\ny_array_lift_no_recursion = ");
        System.out.print(Arrays.deepToString(pos_array.get(1).toArray()));
    }

    public static void main(String[] args) {
        test_magnus_recursion();
        test_drag_no_recursion();
        test_lift_no_recursion();
        test_drag_recursion();
    }
}
