package problem;

/**
 * The possible tires
 */
public enum Tire {
    ALL_TERRAIN,
    MUD,
    LOW_PROFILE,
    PERFORMANCE;

    private String text;

    static {
        ALL_TERRAIN.text = "all-terrain";
        MUD.text = "mud";
        LOW_PROFILE.text = "low-profile";
        PERFORMANCE.text = "performance";
    }

    public String asString() {
        return text;
    }
}
