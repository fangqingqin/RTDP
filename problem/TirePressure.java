package problem;

/**
 * Enum class for the possible tire pressures allowed in problem
 */
public enum TirePressure {
    FIFTY_PERCENT,
    SEVENTY_FIVE_PERCENT,
    ONE_HUNDRED_PERCENT;

    private String text;

    static {
        FIFTY_PERCENT.text = "50%";
        SEVENTY_FIVE_PERCENT.text = "75%";
        ONE_HUNDRED_PERCENT.text = "100%";
    }

    public String asString() {
        return text;
    }
}
