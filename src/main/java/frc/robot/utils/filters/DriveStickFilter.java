package frc.robot.utils.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveStickFilter implements Filter {
    private double MaxSpeed;

    private DeadbandFilter deadbandFilter;
    private SlewRateLimiter slewRateFilter;
        
            public DriveStickFilter(double MaxSpeed, double slewRate, double deadband) {
                this.MaxSpeed = MaxSpeed;
                
                setDeadband(deadband);
                setSlewRate(slewRate);
        }
    
        @Override
        public double filter(double rawAxis) {
            double processedAxis = deadbandFilter.filter(rawAxis);
            return slewRateFilter.calculate(processedAxis);
        }
    
    
        public void setDeadband(double deadband) {
            this.deadbandFilter = new DeadbandFilter(deadband);
        }
    
        public void setSlewRate(double slewRate) {
            this.slewRateFilter = new SlewRateLimiter(slewRate);
        }
        public void setMaxSpeed(double newMaxSpeed)
        {
            this.MaxSpeed = newMaxSpeed;
        }
    
}