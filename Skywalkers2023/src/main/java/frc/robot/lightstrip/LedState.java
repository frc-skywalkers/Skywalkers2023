package frc.robot.lightstrip;

public class LedState {
    private int red;
    private int green;
    private int blue;
    private String effect;

    public LedState(int kRed, int kGreen, int kBlue, String kEffect) {
        red = kRed;
        green = kGreen;
        blue = kBlue;
        effect = kEffect;
    }

    public double[] getState() {
        double[] state = {red, green, blue};
        return state; 
    }

    public int getRed() {
        return red;
    }

    public int getBlue() {
        return blue;
    }

    public int getGreen() {
        return green;
    }

    public String getEffect() {
        return effect;
    }

    public boolean compare(LedState state) {
        return (state.getRed() == getRed() && state.getGreen() == getGreen() && state.getBlue() == getBlue() && state.getEffect() == getEffect());
    }
}
