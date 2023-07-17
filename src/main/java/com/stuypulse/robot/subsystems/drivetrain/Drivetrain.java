package com.stuypulse.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase{
    
    public static final Drivetrain instance;

    static {
        instance = new DrivetrainImpl();
    }

    public static Drivetrain getInstance() {
        return instance;
    }

    public abstract void arcadeDrive(double speed, double rotation);

    public abstract void setLowGear();
    public abstract void setHighGear();

    public abstract boolean isStalling();
}
