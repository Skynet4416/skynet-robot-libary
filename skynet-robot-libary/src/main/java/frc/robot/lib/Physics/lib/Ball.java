package frc.robot.lib.Physics.lib;

import java.util.ArrayList;
import java.util.List;

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

    public void calc_forces() {
        Vector velocity_direction = state.velocity.divide(state.velocity.getMagnitude());
        add_force(Constants.gravitational_acceleration.multiply(mass)); // gravity
        add_force(velocity_direction.multiply(-0.5).multiply(Constants.density_of_air)
                .multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(), 2))); // drag =
                                                                                                             // -1/2 *
                                                                                                             // Cd * p *
                                                                                                             // A * v^2
                                                                                                             // *v(vector)/abs(v)
        Vector axis_of_rotation = state.rotational_velocity.divide(state.rotational_velocity.getMagnitude());
        Scalar lift_coeficent = radius.multiply(state.rotational_velocity.getMagnitude())
                .divide(state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v which
                                                        // is S
        System.out.println(lift_coeficent.getMagnitude());
        add_force(velocity_direction.crossProduct3D(axis_of_rotation).multiply(0.5).multiply(lift_coeficent)
                .multiply(Constants.density_of_air).multiply(cross_section_area_of_ball)
                .multiply(Math.pow(state.velocity.getMagnitude(), 2))); // magnus = 1/2 cl * p * A * v^2 *
                                                                        // (v(vector)/abd(v) X
                                                                        // rotational_velocity(vector)/abs(rotational_velocity))
    }

    public static void main(String[] args) {
        Ball ball = new Ball(0.27, 0.12065, 0.47, 0.2);
        ball.set_started_velocity(new Vector(7.07106781187, 7.07106781187, 0.0));
        ball.set_rotational_velocity(new Vector(0.0, 0.0, -88.1850568421));
        ball.set_position(new Vector(0.0, 0.1, 0.0));
        ball.state.save_to_list();
        ball.calc_forces();
        ball.before_before_state = new State(ball.state);
        List<double[]> pos_array = new ArrayList<double[]>();
        // for(int i = 0; i<1; i++)
        // {
        // ball.calc_forces();
        // ball.update_position();
        // pos_array.add(new
        // double[]{ball.get_position().getComponent(1),ball.get_position().getComponent(2)});
        // }
        System.out.println(pos_array);
    }
}
