package frc.robot.lib.Meth;

import java.util.ArrayList;
import com.github.iprodigy.physics.util.vector.Vector;

import frc.robot.lib.Physics.lib.base.State;

public class Target {
    public final double x_pos;
    public final double y_pos;
    public final double z_pos;
    public final double x_size;
    public final double y_size;
    public final double z_size;
    public final double minimum_entry_angle;
    public final double maximum_entry_angle;
    public final double minimum_entry_velocity;
    public final double maximum_entry_velocity;

    public Target(Vector position, Vector size, double maximum_entry_angle, double minimum_entry_angle,
            double maximum_entry_velocity, double minimum_entry_velocity) {
        this.x_pos = position.getComponent(0);
        this.y_pos = position.getComponent(1);
        this.z_pos = position.getComponent(2);
        this.x_size = size.getComponent(0);
        this.y_size = size.getComponent(1);
        this.z_size = size.getComponent(2);
        this.maximum_entry_angle = maximum_entry_angle;
        this.minimum_entry_angle = minimum_entry_angle;
        this.maximum_entry_velocity = maximum_entry_velocity;
        this.minimum_entry_velocity = minimum_entry_velocity;
    }

    // private double LineFunction(double x, double slope, double x1, double y1) {
    // return (slope * x) - (slope * x1) + y1;
    // }

    public Boolean check(ArrayList<State> states) {
        Segment segment = new Segment(0.0, 0.0, 0.0, 0.0); // LX LY CX CY
        double x_max = this.x_pos + x_size / 2.0;
        double x_min = this.x_pos - x_size / 2.0;
        double y_max = this.y_pos + y_size / 2.0;
        double y_min = this.y_pos - y_size / 2.0;

        for (int index = 0; index < states.size(); index++) {
            segment.LX = segment.CX;
            segment.LY = segment.CY;
            segment.CX = states.get(index).position.getComponent(0);
            segment.CY = states.get(index).position.getComponent(1);

            double angle_deg = Math.toDegrees(Math.atan2(segment.LY - segment.CY, segment.LX - segment.CX));
            double velocity = new Vector(segment.CX - segment.LX, segment.CY - segment.LY).getMagnitude();

            if (segment.CX >= x_min && segment.LX <= x_max) {
                if (segment.LY >= y_min && segment.CY <= y_max) {
                    if (true) { // Z AXIS TBD
                        if (angle_deg >= minimum_entry_angle && angle_deg <= maximum_entry_angle) {
                            if (velocity >= minimum_entry_velocity && velocity <= maximum_entry_velocity)
                                return true;
                        }
                    }
                }
            }
        }
        return false;
    }
}
