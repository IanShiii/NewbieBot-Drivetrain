package com.stuypulse.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Drivetrain.Encoders;
import com.stuypulse.robot.constants.Settings.Drivetrain.Stalling;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DrivetrainImpl extends Drivetrain{
    
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    private Gear gear;
    private final DoubleSolenoid gearShift;
    private final DifferentialDrive drivetrain;

    private final Encoder leftGrayhill;
    private final Encoder rightGrayhill;

    public static enum Gear {
        HIGH(Value.kReverse),
        LOW(Value.kForward);

        private final Value value;

        private Gear(Value value) {
            this.value = value;
        }
    }
    
    public DrivetrainImpl() {

        leftMotors =
            new CANSparkMax[] {
                new CANSparkMax(Ports.Drivetrain.LEFT_BOTTOM, null),
                new CANSparkMax(Ports.Drivetrain.LEFT_MIDDLE, null),
                new CANSparkMax(Ports.Drivetrain.LEFT_TOP, null)
            }; 

        rightMotors =
            new CANSparkMax[] {
                new CANSparkMax(Ports.Drivetrain.RIGHT_BOTTOM, null),
                new CANSparkMax(Ports.Drivetrain.RIGHT_MIDDLE, null),
                new CANSparkMax(Ports.Drivetrain.RIGHT_TOP, null)
            }; 
        
        drivetrain =
            new DifferentialDrive(
                    new MotorControllerGroup(leftMotors),
                    new MotorControllerGroup(rightMotors));
        
        gearShift =
            new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM, 
                Ports.Drivetrain.GEAR_SHIFT_FORWARD,
                Ports.Drivetrain.GEAR_SHIFT_REVERSE);
        
        leftGrayhill = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B);
        rightGrayhill = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);
        setGrayhillDistancePerPulse(Encoders.GRAYHILL_DISTANCE_PER_PULSE);

        setMotorConfig(Motors.Drivetrain.LEFT, Motors.Drivetrain.RIGHT);
        setHighGear();
    }

    /********************
     * DRIVING COMMANDS *
     ********************/

    // Stops drivetrain from moving
    public void stop() {
        drivetrain.stopMotor();
    }

    // Drives using arcade drive
    @Override
    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation, false);
    }

    /***********************
     * MOTOR CONFIGURATION *
     ***********************/

    private void setMotorConfig(Motors.Config left, Motors.Config right) {
        leftGrayhill.setReverseDirection(
                Settings.Drivetrain.Encoders.GRAYHILL_INVERTED ^ left.INVERTED);
        for (CANSparkMax motor : leftMotors) {
            left.configure(motor);
        }

        rightGrayhill.setReverseDirection(
                Settings.Drivetrain.Encoders.GRAYHILL_INVERTED ^ right.INVERTED);
        for (CANSparkMax motor : rightMotors) {
            right.configure(motor);
        }
    }
    
    private void setGrayhillDistancePerPulse(double distance) {
        rightGrayhill.setDistancePerPulse(distance);
        rightGrayhill.reset();

        leftGrayhill.setDistancePerPulse(distance);
        leftGrayhill.reset();
    }

    /*****************
     * Gear Shifting *
     *****************/

    // Gets the current gear the robot is in
    public Gear getGear() {
        return gear;
    }

    // Sets the current gear the robot is in
    private void setGear(Gear gear) {
        gearShift.set(gear.value);
        this.gear = gear;
    }

    // Sets robot into low gear]
    @Override
    public void setLowGear() {
        setGear(Gear.LOW);
    }

    // Sets robot into high gear
    @Override
    public void setHighGear() {
        setGear(Gear.HIGH);
    }

    /*********************
     * ENCODER FUNCTIONS *
     *********************/

    // Distance
    public double getLeftDistance() {
        return leftGrayhill.getDistance();
    }

    public double getRightDistance() {
        return rightGrayhill.getDistance();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // Velocity
    public double getLeftVelocity() {
        return leftGrayhill.getRate();
    }

    public double getRightVelocity() {
        return rightGrayhill.getRate();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /*******************
     * STALL DETECTION *
     *******************/

    public double getLeftCurrentAmps() {
        double amps = 0.0;

        for (CANSparkMax motor : leftMotors) {
            amps += Math.abs(motor.getOutputCurrent());
        }

        return amps / leftMotors.length;
    }

    public double getRightCurrentAmps() {
        double amps = 0.0;

        for (CANSparkMax motor : rightMotors) {
            amps += Math.abs(motor.getOutputCurrent());
        }

        return amps / rightMotors.length;
    }

    public double getCurrentAmps() {
        return (getLeftCurrentAmps() + getRightCurrentAmps()) / 2.0;
    }

    public boolean isLeftStalling() {
        boolean highGear = getGear() == Gear.HIGH;
        boolean current = getLeftCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(leftMotors[0].get()) > Stalling.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(getLeftVelocity()) < Stalling.SCIBORGS_THRESHOLD;
        return highGear && (current || output) && velocity;
    }

    public boolean isRightStalling() {
        boolean highGear = getGear() == Gear.HIGH;
        boolean current = getRightCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(rightMotors[0].get()) > Stalling.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(getRightVelocity()) < Stalling.SCIBORGS_THRESHOLD;
        return highGear && (current || output) && velocity;
    }

    @Override
    public boolean isStalling() {
        return isLeftStalling() || isRightStalling();
    }
}
