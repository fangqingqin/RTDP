package problem;

/**
 * List of possible actions
 */
public enum ActionType {
    MOVE,                       // A1
    CHANGE_CAR,                 // A2
    CHANGE_DRIVER,              // A3
    CHANGE_TIRES,               // A4
    ADD_FUEL,                   // A5
    CHANGE_PRESSURE,            // A6
    CHANGE_CAR_AND_DRIVER,      // A7
    CHANGE_TIRE_FUEL_PRESSURE;  // A8

    private int actionNo;

    static {
        MOVE.actionNo = 1;
        CHANGE_CAR.actionNo = 2;
        CHANGE_DRIVER.actionNo = 3;
        CHANGE_TIRES.actionNo = 4;
        ADD_FUEL.actionNo = 5;
        CHANGE_PRESSURE.actionNo = 6;
        CHANGE_CAR_AND_DRIVER.actionNo = 7;
        CHANGE_TIRE_FUEL_PRESSURE.actionNo = 8;
    }

    public int getActionNo() {
        return actionNo;
    }
}
