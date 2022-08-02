package frc.robot.lib.Physics.lib;

import com.github.iprodigy.physics.util.abstraction.Scalar;
import com.github.iprodigy.physics.util.vector.Vector;

import frc.robot.lib.Physics.lib.base.PhysicalObjectBase;
import frc.robot.lib.Physics.lib.base.State;
import frc.robot.lib.Physics.Constants;

public class Ball extends PhysicalObjectBase{
    protected Scalar radius;
    protected Scalar drag_constant;
    protected Scalar magnus_constant;
    protected Scalar cross_section_area_of_ball;
    public Ball(double mass, double radius, double drag_constant,double estemation_resulotion)
    {
        super.estemation_resulotion = () ->estemation_resulotion;
        super.mass=() -> mass;
        super.state = new State();
        this.drag_constant = () -> drag_constant;
        this.radius = () -> radius;
        cross_section_area_of_ball = this.radius.multiply(this.radius).multiply(Math.PI);
    }
    public void set_position(Vector position)
    {
        super.state.position = position;
    }
    public void set_started_velocity(Vector velocity)
    {
        super.state.velocity = velocity;
    }
    public void calc_forces()
    {
        Vector velocity_direction = state.velocity.divide(state.velocity.getMagnitude()); 
        super.add_force(Constants.gravitational_acceleration.multiply(mass)); //gravity
        super.add_force(velocity_direction.multiply(-1/2).multiply(Constants.density_of_air).multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(),2))); // drag  = -1/2 * Cd * p * A * v^2 *v(vector)/abs(v)
        Vector axis_of_rotation =  state.rotational_velocity.divide(state.rotational_velocity.getMagnitude());
        Scalar lift_coeficent = radius.multiply(state.rotational_velocity.getMagnitude()).divide(state.velocity.getMagnitude()); // if cd is close to 0.5 then cl = R*rotational_velocity/v which is S
        super.add_force(velocity_direction.crossProduct3D(axis_of_rotation).multiply(1/2).multiply(lift_coeficent).multiply(Constants.density_of_air).multiply(cross_section_area_of_ball).multiply(Math.pow(state.velocity.getMagnitude(),2))); // magnus = 1/2 cl * p * A * v^2 * (v(vector)/abd(v) X rotational_velocity(vector)/abs(rotational_velocity))
    }
    
}
