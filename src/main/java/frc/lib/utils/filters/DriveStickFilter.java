package frc.lib.utils.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveStickFilter implements Filter {
    private DeadbandFilter deadbandFilter;
    private SlewRateLimiter slewRateFilter;
        
        public DriveStickFilter(double slewRate, double deadband) {                
            setDeadband(deadband);
            setSlewRate(slewRate);
        }
        public DriveStickFilter(double slewRate, DeadbandFilter deadbandFilter) {                
            this.deadbandFilter = deadbandFilter;
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
}