package frc.robot.lib.Meth;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;

import com.github.iprodigy.physics.util.vector.Vector;

import edu.wpi.first.math.util.Units;
import frc.robot.lib.Physics.lib.Ball;
import frc.robot.lib.Physics.lib.base.State;

public final class shooter_optimiztion {
    public enum OptimizationType {
        MAXIMIZE,
        MINIMIZE,
        IGNORE
    }

    public static void states_to_pos(ArrayList<State> states) {
        ArrayList<Double> x = new ArrayList<Double>();
        ArrayList<Double> y = new ArrayList<Double>();
        for (State state : states) {
            x.add(state.position.getComponent(0));
            y.add(state.position.getComponent(1));

        }
        System.out.print("\n\n\nx_array = ");
        System.out.print(Arrays.deepToString(x.toArray()));
        System.out.print("\n\n\ny_array = ");
        System.out.println(Arrays.deepToString(y.toArray()));

    }

    private static final Integer failure_tolerance = 200;
    private static final Integer max_attempts = 9999;
    // Target hub = new Target(new Vector(16.0, 2.7178, 1.0), new Vector(1.22, 0.5,
    // 1.0), 999, -999, 999, -999);

    interface BooleanInterface {
        Boolean run(Double angle, Double rpm, Double ratio);
    }

    public static Double Lerp(Double A, Double B, Double Percent) {
        return A + (B - A) * Percent;
    }

    public static void optimize(Ball projectile, Target target, // PHYSICALS
            OptimizationType shooting_angle, OptimizationType shooting_RPM, OptimizationType spin_dierction, // INITIALS
            OptimizationType entrance_velocity, OptimizationType entrance_angle, OptimizationType trajectory_length, // GUIDES
            Double Max_Angle, Double Min_Angle, Double Max_RPM, Double Min_RPM) { // CONSTRAINTS

        Instant starts = Instant.now();

        double TopinDiameter = 4;
        double TopmDiameter = Units.inchesToMeters(TopinDiameter);
        double TopCircumference = (TopmDiameter) * Math.PI;

        double BottominDiameter = 4;
        double BottommDiameter = Units.inchesToMeters(BottominDiameter);
        double BottomCircumference = (BottommDiameter) * Math.PI;

        ///////////////////////////
        Integer Failures = 0;

        Double Angle_Resolution = 1.0; // DEGREES
        Double Rotation_Ratio_Resolution = 0.1; // PERCENTAGE
        Double RPM_Resolution = 25.0; // RPM

        Double Initial_Angle = (shooting_angle != OptimizationType.MAXIMIZE) ? Max_Angle : Min_Angle;
        Double Angle_Increment = (shooting_angle != OptimizationType.MAXIMIZE) ? -Angle_Resolution : Angle_Resolution;
        Double Current_Angle = Initial_Angle;

        Double Initial_Rotation_Ratio = (spin_dierction != OptimizationType.MAXIMIZE) ? 1 - (Min_RPM / Max_RPM)
                : (Min_RPM / Max_RPM);
        Double Rotation_Ratio_Increment = (spin_dierction != OptimizationType.MAXIMIZE) ? -Rotation_Ratio_Resolution
                : Rotation_Ratio_Resolution;
        Double Current_Rotation_Ratio = (spin_dierction == OptimizationType.IGNORE) ? -1.0 : Initial_Rotation_Ratio;

        Double Initial_RPM = (shooting_RPM != OptimizationType.MAXIMIZE) ? Max_RPM : Min_RPM;
        Double RPM_Increment = (shooting_RPM != OptimizationType.MAXIMIZE) ? -RPM_Resolution : RPM_Resolution;
        Double Current_RPM = Initial_RPM;

        BooleanInterface full_check = new BooleanInterface() {
            public Boolean run(Double angle, Double rpm, Double ratio) {
                double TopRPM = (spin_dierction != OptimizationType.IGNORE) ? rpm * ratio : rpm;
                double TopRPS = TopRPM / 60.0;
                double BottomRPM = (spin_dierction != OptimizationType.IGNORE) ? rpm * (1 - ratio) : rpm;
                double BottomRPS = BottomRPM / 60.0;

                double muzzle_velocity = (TopRPS * TopCircumference + BottomRPS * BottomCircumference) / 2.0; // SURFACE
                                                                                                              // SPEED
                double angle_rads = angle; // IN RADIANS

                double TopRads = 0.104719755 * TopRPM;
                double BottompRads = 0.104719755 * BottomRPM;

                Vector started_velocity = new Vector(muzzle_velocity * Math.sin(angle_rads),
                        muzzle_velocity * Math.cos(angle_rads), 0.0); // m/s
                Vector started_rotational_velocity = new Vector(0.0, 0.0,
                        (TopRads * TopmDiameter / 2 - BottompRads * BottommDiameter / 2)
                                / (2 * projectile.get_radius())); // radians

                projectile.set_started_velocity(started_velocity);
                projectile.set_rotational_velocity(started_rotational_velocity);

                ArrayList<State> states = projectile.simulate_ball(false);
                Boolean result = target.check(states);

                if (result) {
                    states_to_pos(states);
                }

                projectile.set_position(new Vector(0.0, 0.1, 0.0));
                return result;
            };
        };

        System.out.println("STARTED!");
        Integer results = 0;
        // for (Integer Attempts = 0; Attempts < max_attempts; Attempts++) {
        // CurrentFound = false;
        // System.out.println(Attempts + " " + Current_Angle + " " + Current_RPM + " " +
        // Current_Rotation_Ratio);

        Integer Attempts = 0;

        Double BestTopRPM = 0.0;
        Double BestBottomRPM = 0.0;
        Double BestAngle = 0.0;

        Current_Angle = Initial_Angle;
        while (shooting_angle == OptimizationType.MAXIMIZE ? Current_Angle <= Max_Angle : Current_Angle >= Min_Angle) {
            Current_RPM = Initial_RPM;
            while (shooting_RPM == OptimizationType.MAXIMIZE ? Current_RPM <= Max_RPM : Current_RPM >= Min_RPM) {
                Current_Rotation_Ratio = Initial_Rotation_Ratio;
                do {

                    Attempts++;
                    if (full_check.run(Current_Angle, Current_RPM, Current_Rotation_Ratio)) {
                        Failures = 0;
                        results++;
                        BestTopRPM = (spin_dierction != OptimizationType.IGNORE)
                                ? Current_RPM * Current_Rotation_Ratio
                                : Current_RPM;

                        BestBottomRPM = (spin_dierction != OptimizationType.IGNORE)
                                ? Current_RPM * (1 - Current_Rotation_Ratio)
                                : Current_RPM;
                        BestAngle = Current_Angle;

                        // System.out.println("HIT> " + Current_RPM + " " + Current_Angle);
                    }
                    Current_Rotation_Ratio += Rotation_Ratio_Increment;
                } while ((spin_dierction == OptimizationType.MAXIMIZE
                        ? Current_Rotation_Ratio <= 1.0 - (Min_RPM / Max_RPM)
                        : Current_Rotation_Ratio >= (Min_RPM / Max_RPM)) && spin_dierction != OptimizationType.IGNORE);
                Current_RPM += RPM_Increment;
            }
            Current_Angle += Angle_Increment;
            Failures++;
        }

        // Current_Rotation_Ratio += Rotation_Ratio_Increment;

        // if (!CurrentFound)
        // if (full_check.run(Current_Angle, Current_RPM, Current_Rotation_Ratio)) {
        // Failures = 0;
        // AllFound = true;
        // CurrentFound = true;
        // results++;

        // System.out.println("HIT!");
        // }

        System.out.println((double) Duration.between(starts, Instant.now()).toMillis() / 1000);
        System.out.println("DONE, " + results + " results. " + Attempts + " attempts.");
        System.out.println("TRPM " + BestTopRPM + "\nBRPM " + BestBottomRPM + "\nANGLE " + BestAngle);

    }
}
