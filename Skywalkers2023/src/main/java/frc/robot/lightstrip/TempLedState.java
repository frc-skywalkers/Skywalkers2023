package frc.robot.lightstrip;

public class TempLedState extends LedState {
    private int seconds;

    public TempLedState(int kRed, int kGreen, int kBlue, String kEffect, int kSeconds) {
        super(kRed, kGreen, kBlue, kEffect);
        
        seconds = kSeconds;
    }
    
    public int getSeconds() {
        return seconds;
    }
}
