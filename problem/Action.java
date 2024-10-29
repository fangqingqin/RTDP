package problem;

/**
 * An action in the MDP
 */
public class Action {

    /** The type of action **/
    private ActionType actionType;
    /** car type **/
    private String carType;
    /** driver type **/
    private String driverType;
    /** tire model **/
    private Tire tireModel;
    /** fuel to add **/
    private int fuel;
    /** tire pressure change **/
    private TirePressure tirePressure;
    /** String representation of action **/
    private String text;

    /**
     * Constructor for the MOVE action
     *
     * @param actionType MOVE
     */
    public Action(ActionType actionType) {
        if (actionType == ActionType.MOVE) {
            this.actionType = actionType;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be MOVE");
        }
        text = "A" + actionType.getActionNo();
    }

    /**
     * Constructor for CHANGE_CAR or CHANGE_DRIVER action.
     *
     * @param actionType CHANGE_CAR or CHANGE_DRIVER
     * @param type a car type or driver type
     */
    public Action(ActionType actionType, String type) {
        if (actionType == ActionType.CHANGE_CAR) {
            this.actionType = actionType;
            carType = type;
        } else if (actionType == ActionType.CHANGE_DRIVER) {
            driverType = type;
            this.actionType = actionType;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be CHANGE_CAR or CHANGE_DRIVER");
        }
        text = "A" + actionType.getActionNo();
        text += ":" + type;
    }

    /**
     * Constructor for CHANGE_TIRES action.
     *
     * @param actionType CHANGE_T
     * @param tireModel tire model to change to
     */
    public Action(ActionType actionType, Tire tireModel) {
        if (actionType == ActionType.CHANGE_TIRES) {
            this.actionType = actionType;
            this.tireModel = tireModel;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be CHANGE_TIRES");
        }
        text = "A" + actionType.getActionNo();
        text += ":" + tireModel.asString();
    }

    /**
     * Constructor for ADD_FUEL action.
     *
     * @param actionType ADD_FUEL
     * @param fuel amount of fuel to add
     */
    public Action(ActionType actionType, int fuel) {
        if (actionType == ActionType.ADD_FUEL) {
            this.actionType = actionType;
            if (fuel < ProblemSpec.FUEL_MIN || fuel > ProblemSpec.FUEL_MAX) {
                throw new IllegalArgumentException("Fuel amount must be: 0 <= fuel <= 50");
            }
            this.fuel = fuel;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be ADD_FUEL");
        }
        text = "A" + actionType.getActionNo();
        text += ":" + fuel;
    }

    /**
     * Constructor for CHANGE_PRESSURE action.
     *
     * @param actionType CHANGE_PRESSURE
     * @param tirePressure the new tire pressure
     */
    public Action(ActionType actionType, TirePressure tirePressure) {
        if (actionType == ActionType.CHANGE_PRESSURE) {
            this.actionType = actionType;
            this.tirePressure = tirePressure;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be CHANGE_PRESSURE");
        }
        text = "A" + actionType.getActionNo();
        text += ":" + tirePressure.asString();
    }

    /**
     * Constructor for CHANGE_CAR_AND_DRIVER action.
     *
     * @param actionType CHANGE_CAR_AND_DRIVER
     * @param carType new car type
     * @param driverType new driver
     */
    public Action(ActionType actionType, String carType, String driverType) {
        if (actionType == ActionType.CHANGE_CAR_AND_DRIVER) {
            this.actionType = actionType;
            this.carType = carType;
            this.driverType = driverType;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be CHANGE_CAR_AND_DRIVER");
        }
        text = "A" + actionType.getActionNo();
        text += ":" + carType + ":" + driverType;
    }

    /**
     * Constructor for CHANGE_TIRE_FUEL_PRESSURE action
     *
     * @param actionType CHANGE_TIRE_FUEL_PRESSURE
     * @param tireModel the new tire model
     * @param fuel the fuel to add
     * @param tirePressure the new tire pressure
     */
    public Action(ActionType actionType, Tire tireModel, int fuel, TirePressure tirePressure) {
        if (actionType == ActionType.CHANGE_TIRE_FUEL_PRESSURE) {
            this.actionType = actionType;
            this.tireModel = tireModel;
            if (fuel < ProblemSpec.FUEL_MIN || fuel > ProblemSpec.FUEL_MAX) {
                throw new IllegalArgumentException("Fuel amount must be: 0 <= fuel <= 50");
            }
            this.fuel = fuel;
            this.tirePressure = tirePressure;
        } else {
            throw new IllegalArgumentException("Action type for this constructor must be CHANGE_TIRE_FUEL_PRESSURE");
        }
        text = "A" + actionType.getActionNo();
        text += ":" + tireModel.asString() + ":" + fuel + ":" + tirePressure.asString();
    }

    public ActionType getActionType() {
        return actionType;
    }

    public String getCarType() {
        return carType;
    }

    public String getDriverType() {
        return driverType;
    }

    public Tire getTireModel() {
        return tireModel;
    }

    public int getFuel() {
        return fuel;
    }

    public TirePressure getTirePressure() {
        return tirePressure;
    }

    public String getText() {
        return text;
    }
}
