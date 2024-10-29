package problem;

import java.util.ArrayList;
import java.util.List;

/**
 * This class provides the properties for a given level
 **/
public class Level {

    /** Level number **/
    private int levelNumber;
    /** List of terrain types present in level **/
    private List<Terrain> terrainTypes;
    /** List of action types possible in level **/
    private List<ActionType> availableActions;

    /**
     * Load the level number
     *
     * @param levelNumber number of level (1 to 5)
     */
    public Level(int levelNumber) {
        loadLevel(levelNumber);
    }

    public int getLevelNumber() {
        return levelNumber;
    }

    /**
     * Returns the number of terrain types for the level
     *
     * @return NT
     */
    public int get_NT() {
        return terrainTypes.size();
    }

    public List<Terrain> getTerrainTypes() {
        return terrainTypes;
    }

    public List<ActionType> getAvailableActions() {
        return availableActions;
    }

    public boolean isValidActionForLevel(ActionType a) {
        return availableActions.contains(a);
    }

    private void loadLevel(int levelNumber) {
        this.levelNumber = levelNumber;
        terrainTypes = new ArrayList<>();
        availableActions = new ArrayList<>();

        switch (levelNumber) {
            case 1:
                loadTerrain(2);
                loadActions(4);
                break;
            case 2:
                loadTerrain(4);
                loadActions(6);
                break;
            case 3:
                loadTerrain(8);
                loadActions(6);
                break;
            case 4:
                loadTerrain(8);
                loadActions(7);
                break;
            case 5:
                loadTerrain(8);
                loadActions(8);
                break;
            default:
                System.out.println("Invalid level number " + levelNumber);
                System.exit(1);
        }
    }

    private void loadTerrain(int number) {
        if (number == 2) {
            terrainTypes.add(Terrain.DIRT);
            terrainTypes.add(Terrain.ASPHALT);
        } else if (number == 4) {
            terrainTypes.add(Terrain.DIRT_STRAIGHT);
            terrainTypes.add(Terrain.DIRT_SLALOM);
            terrainTypes.add(Terrain.ASPHALT_STRAIGHT);
            terrainTypes.add(Terrain.ASPHALT_SLALOM);
        } else {
            terrainTypes.add(Terrain.DIRT_STRAIGHT_HILLY);
            terrainTypes.add(Terrain.DIRT_STRAIGHT_FLAT);
            terrainTypes.add(Terrain.DIRT_SLALOM_HILLY);
            terrainTypes.add(Terrain.DIRT_SLALOM_FLAT);
            terrainTypes.add(Terrain.ASPHALT_STRAIGHT_HILLY);
            terrainTypes.add(Terrain.ASPHALT_STRAIGHT_FLAT);
            terrainTypes.add(Terrain.ASPHALT_SLALOM_HILLY);
            terrainTypes.add(Terrain.ASPHALT_SLALOM_FLAT);
        }
    }

    private void loadActions(int maxAction) {
        if (maxAction >= 4) {
            availableActions.add(ActionType.MOVE);
            availableActions.add(ActionType.CHANGE_CAR);
            availableActions.add(ActionType.CHANGE_DRIVER);
            availableActions.add(ActionType.CHANGE_TIRES);
        }
        if (maxAction >= 6) {
            availableActions.add(ActionType.ADD_FUEL);
            availableActions.add(ActionType.CHANGE_PRESSURE);
        }
        if (maxAction >= 7) {
            availableActions.add(ActionType.CHANGE_CAR_AND_DRIVER);
        }
        if (maxAction >= 8) {
            availableActions.add(ActionType.CHANGE_TIRE_FUEL_PRESSURE);
        }
    }
}
