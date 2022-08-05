package frc.robot.lib.Flywheel.Subsystem;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.lib.base.MotorControllers.MotorControllerBase;
import frc.robot.lib.base.Subsytems.MySubsytemBase;
import frc.robot.lib.Physics.Constants;

public class SingleFlywheel extends MySubsytemBase{
    protected ArrayList<Pair<MotorControllerBase,DCMotor>> left_side;
    protected ArrayList<Pair<MotorControllerBase,DCMotor>> right_side;
    protected double wheel_radius_meters;



    public SingleFlywheel(ArrayList<Pair<MotorControllerBase,DCMotor>> left_side, ArrayList<Pair<MotorControllerBase,DCMotor>> right_side,double wheel_radius_meters)
    {
        this.left_side = left_side;
        this.right_side = right_side;
        this.wheel_radius_meters = wheel_radius_meters;
    }
    
    


    

}
