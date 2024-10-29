package problem;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

/**
 * This class represents the problem detailed in the assignment spec.
 * It contains functionality to parse the input file and load it into class
 * variable (see comments for each variable for more info).
 */
public class ProblemSpec {

    /** min and max values for car fuel level **/
    public static final int FUEL_MIN = 0;
    public static final int FUEL_MAX = 50;
    /** car move range [-4, 5] **/
    public static final int CAR_MIN_MOVE = -4;
    public static final int CAR_MAX_MOVE = 5;
    public static final int SLIP = 6;
    public static final int BREAKDOWN = 7;
    public static final int CAR_MOVE_RANGE = 12;
    /** number of different tyres **/
    public static final int NUM_TYRE_MODELS = 4;
    /** number of different levels of tyre pressure **/
    public static final int TIRE_PRESSURE_LEVELS = 3;
    /** Max slip probability **/
    public static final double MAX_SLIP_PROBABILITY = 0.9;

    /** The level of the game **/
    private Level level;
    /** Discount factor **/
    private double discountFactor;
    /** Time to recover from a slip **/
    private int slipRecoveryTime;
    /** Breakdown repair time **/
    private int repairTime;
    /** The number of cells in map **/
    private int N;
    /** The maximum number of time-steps allowed for reaching goal **/
    private int maxT;
    /** Number of terrain types **/
    private int NT;
    /** The environment map as a 1D array of terrains in order **/
    private Terrain[] environmentMap;
    /** The terrain map which maps terrains to their cell indices on the
     * environment map */
    private LinkedHashMap<Terrain, List<Integer>> terrainMap;
    /** Ordering of terrain as they appear in input **/
    private List<Terrain> terrainOrder;
    /** Number of car types **/
    private int CT;
    /** Car probability mapping **/
    private LinkedHashMap<String, double[]> carMoveProbability;
    /** Ordering of car types as they appear in input **/
    private List<String> carOrder;
    /** Number of drivers **/
    private int DT;
    /** Driver to probability mapping **/
    private LinkedHashMap<String, double[]> driverMoveProbability;
    /** Ordering of drivers as they appear in input **/
    private List<String> driverOrder;
    /** Tyre model to probability mapping **/
    private LinkedHashMap<Tire, double[]> tireModelMoveProbability;
    /** Ordering of tire models as they appear in input **/
    private List<Tire> tireOrder;
    /** Fuel usage matrix
     * Size is NT rows * CT columns
     * Each row, i, represents the ith terrain type
     * Each column, j, represents the jth Car type */
    private int[][] fuelUsage;
    /** Slip probability for each terrain for 50% tire pressure **/
    private double[] slipProbability;

    /**
     * Load problem spec from input file
     *
     * @param fileName path to input file
     * @throws IOException if can't find file or there is a format error
     */
    public ProblemSpec(String fileName) throws IOException {
        loadProblem(fileName);
    }

    /**
     * Loads a problem from a problem text file.
     *
     * @param fileName
     *              the path of the text file to load.
     * @throws IOException
     *              if the text file doesn't exist or doesn't meet the
     *              assignment specifications.
     */
    private void loadProblem(String fileName) throws IOException {
        String line;
        String[] splitLine;
        int lineNo = 0;
        Scanner s;
        try (BufferedReader input = new BufferedReader(new FileReader(fileName))) {

            // 1. line 1
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            int levelNumber = s.nextInt();
            level = new Level(levelNumber);
            s.close();

            // 2. line 2
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            discountFactor = s.nextDouble();
            slipRecoveryTime = s.nextInt();
            repairTime = s.nextInt();
            s.close();

            // 3. line 3
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            N = s.nextInt();
            maxT = s.nextInt();
            s.close();

            // 4. line 4 to (3+NT)
            NT = level.get_NT();
            environmentMap = new Terrain[N];
            terrainMap = new LinkedHashMap<>();
            terrainOrder = new ArrayList<>();
            List<Integer> terrainIndices;
            for (int i = 0; i < NT; i++) {
                line = input.readLine();
                lineNo++;
                splitLine = line.split(":");
                // first part is name of terrain
                Terrain terrain = parseTerrain(splitLine[0], lineNo);
                terrainIndices = parseTerrainCellIndices(splitLine[1]);
                terrainMap.put(terrain, terrainIndices);
                for (Integer j: terrainIndices) {
                    environmentMap[j-1] = terrain;
                }
                terrainOrder.add(terrain);
            }

            // 5. line (3+NT+1)
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            CT = s.nextInt();
            s.close();

            // 6. line (3+NT+2) to (3+NT+2+CT)
            String car;
            carMoveProbability = new LinkedHashMap<>();
            carOrder = new ArrayList<>();
            for (int i = 0; i < CT; i++) {
                line = input.readLine();
                lineNo++;
                car = parseProbLine(line, carMoveProbability);
                carOrder.add(car);
            }

            // 7. Number of drivers line
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            DT = s.nextInt();
            s.close();

            // 8. Driver move probabilities
            String driver;
            driverMoveProbability = new LinkedHashMap<>();
            driverOrder = new ArrayList<>();
            for (int i = 0; i < DT; i++) {
                line = input.readLine();
                lineNo++;
                driver = parseProbLine(line, driverMoveProbability);
                driverOrder.add(driver);
            }

            // 9. Tyre model move probabilities
            Tire tire;
            tireModelMoveProbability = new LinkedHashMap<>();
            tireOrder = new ArrayList<>();
            for (int i = 0; i < NUM_TYRE_MODELS; i++) {
                line = input.readLine();
                lineNo++;
                tire = parseTireModelProbability(line, lineNo,
                        tireModelMoveProbability);
                tireOrder.add(tire);
            }

            // 10. Fuel usage by terrain and car matrix
            fuelUsage = new int[NT][CT];
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            for (int i = 0; i < NT; i++) {
                for (int j = 0; j < CT; j++) {
                    fuelUsage[i][j] = s.nextInt();
                }
            }
            s.close();

            // 11. Slip probability by terrain
            slipProbability = new double[NT];
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            for (int i = 0; i < NT; i++) {
                slipProbability[i] = s.nextDouble();

            }
            s.close();
            input.close();

        } catch (InputMismatchException e) {
            System.out.println(e.getMessage());
            System.exit(1);
        } catch (NoSuchElementException e) {
            System.out.println("Not enough tokens on input file - line " + lineNo);
            System.exit(2);
        } catch (NullPointerException e) {
            System.out.format("Input file - line %d expected, but file ended.", lineNo);
            System.exit(3);
        } catch (FileNotFoundException e) {
            System.out.println(e.getMessage());
            System.exit(4);
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        // 1.
        sb.append("level: ").append(level.getLevelNumber()).append("\n");
        // 2.
        sb.append("discount: ").append(discountFactor).append("\n");
        sb.append("recoverTime: ").append(slipRecoveryTime).append("\n");
        sb.append("repairTime: ").append(repairTime).append("\n");
        // 3.
        sb.append("N: ").append(N).append("\n");
        sb.append("maxT: ").append(maxT).append("\n");
        // 4.
        sb.append("Environment map: [");
        for (int i = 0; i < environmentMap.length; i++) {
            sb.append(environmentMap[i].asString());
            if (i < environmentMap.length - 1)
                sb.append(" | ");
        }
        sb.append("]\n");
        // 5. here lies the corpse of the tutor who started writing this
        // function and realized he has a thesis due. RIP.

        return sb.toString();
    }

    /**
     * Parse a line of the below form and add entry to map:
     *
     *      thingName : p0 p1 p2 ... p11
     *
     * where pi represents probability of ith possible car move distance,
     * starting at -4 upto 5, then slip and breakdown.
     *
     * @param line the line text
     * @param probMap map to add the entry to
     * @return the name of thing
     */
    private String parseProbLine(String line, Map<String, double[]> probMap) {
        String[] splitLine = line.split(":");
        String thingName = splitLine[0];
        Scanner s = new Scanner(splitLine[1]);
        double[] probabilities = new double[CAR_MOVE_RANGE];
        double pSum = 0;
        for (int j = 0; j < CAR_MOVE_RANGE; j++) {
            probabilities[j] = s.nextDouble();
            pSum += probabilities[j];
        }
        probMap.put(thingName, probabilities);
        s.close();

        if (Math.abs(pSum - 1.0) > 0.001) {
            throw new InputMismatchException("Car move probability does not sum to 1.0");
        }

        return thingName;
    }

    /**
     * Parse a line of the below form and add entry to map:
     *
     *      tireModel : p0 p1 p2 ... p11
     *
     * where pi represents probability of ith possible car move distance,
     * starting at -4 upto 5, then slip and breakdown.
     *
     * @param line the line of text
     * @param lineNo the line no
     * @param probMap map to add entry to
     * @return the tire model
     */
    private Tire parseTireModelProbability(String line, int lineNo,
                                           Map<Tire, double[]> probMap) {
        String[] splitLine = line.split(":");
        Tire tireModel = parseTireModel(splitLine[0], lineNo);
        Scanner s = new Scanner(splitLine[1]);
        double[] probabilities = new double[CAR_MOVE_RANGE];
        double pSum = 0;
        for (int j = 0; j < CAR_MOVE_RANGE; j++) {
            probabilities[j] = s.nextDouble();
            pSum += probabilities[j];
        }
        probMap.put(tireModel, probabilities);
        s.close();

        if (Math.abs(pSum - 1.0) > 0.001) {
            throw new InputMismatchException("Car move probability does not sum to 1.0");
        }

        return tireModel;
    }

    /**
     * Parse a terrain and cell index of terrain line from input
     *
     * @param indexText the indeces part of the line text
     * @return list of indices for given terrain
     */
    private List<Integer> parseTerrainCellIndices(String indexText) {

        List<Integer> indices = new ArrayList<>();

        String[] splitText = indexText.split(",");
        String[] splitIndices;
        int start, end;

        for (String s: splitText) {
            splitIndices = s.split("-");

            if (splitIndices.length == 1) {
                indices.add(Integer.parseInt(splitIndices[0]));
            } else if (splitIndices.length == 2) {
                start = Integer.parseInt(splitIndices[0]);
                end = Integer.parseInt(splitIndices[1]);
                for (int i = start; i <= end; i++) {
                    indices.add(i);
                }
            }
            // else empty so no terrain of this type
        }
        return indices;
    }

    private Tire parseTireModel(String tireText, int lineNo) {
        switch (tireText) {
            case "all-terrain":
                return Tire.ALL_TERRAIN;
            case "mud":
                return Tire.MUD;
            case "low-profile":
                return Tire.LOW_PROFILE;
            case "performance":
                return Tire.PERFORMANCE;
            default:
                String errMsg = "Invalid tyre type " + tireText + "on line " + lineNo;
                throw new InputMismatchException(errMsg);
        }
    }

    private Terrain parseTerrain(String terrainText, int lineNo) {
        switch (terrainText) {
            case "dirt":
                return Terrain.DIRT;
            case "asphalt":
                return Terrain.ASPHALT;
            case "dirt-straight":
                return Terrain.DIRT_STRAIGHT;
            case "dirt-slalom":
                return Terrain.DIRT_SLALOM;
            case "asphalt-straight":
                return Terrain.ASPHALT_STRAIGHT;
            case "asphalt-slalom":
                return Terrain.ASPHALT_SLALOM;
            case "dirt-straight-hilly":
                return Terrain.DIRT_STRAIGHT_HILLY;
            case "dirt-straight-flat":
                return Terrain.DIRT_STRAIGHT_FLAT;
            case "dirt-slalom-hilly":
                return Terrain.DIRT_SLALOM_HILLY;
            case "dirt-slalom-flat":
                return Terrain.DIRT_SLALOM_FLAT;
            case "asphalt-straight-hilly":
                return Terrain.ASPHALT_STRAIGHT_HILLY;
            case "asphalt-straight-flat":
                return Terrain.ASPHALT_STRAIGHT_FLAT;
            case "asphalt-slalom-hilly":
                return Terrain.ASPHALT_SLALOM_HILLY;
            case "asphalt-slalom-flat":
                return Terrain.ASPHALT_SLALOM_FLAT;
            default:
                String errMsg = "Invalid terrain type " + terrainText + "on line " + lineNo;
                throw new InputMismatchException(errMsg);
        }
    }

    public Level getLevel() {
        return level;
    }

    public double getDiscountFactor() {
        return discountFactor;
    }

    public int getSlipRecoveryTime() {
        return slipRecoveryTime;
    }

    public int getRepairTime() {
        return repairTime;
    }

    public int getN() {
        return N;
    }

    public int getMaxT() {
        return maxT;
    }

    public int getNT() {
        return NT;
    }

    public Terrain[] getEnvironmentMap() {
        return environmentMap;
    }

    public int getCT() {
        return CT;
    }

    public int getDT() {
        return DT;
    }

    public LinkedHashMap<String, double[]> getCarMoveProbability() {
        return carMoveProbability;
    }

    public List<String> getCarOrder() {
        return carOrder;
    }

    public LinkedHashMap<Terrain, List<Integer>> getTerrainMap() {
        return terrainMap;
    }

    public LinkedHashMap<String, double[]> getDriverMoveProbability() {
        return driverMoveProbability;
    }

    public List<String> getDriverOrder() {
        return driverOrder;
    }

    public LinkedHashMap<Tire, double[]> getTireModelMoveProbability() {
        return tireModelMoveProbability;
    }

    public List<Tire> getTireOrder() {
        return tireOrder;
    }

    public int[][] getFuelUsage() {
        return fuelUsage;
    }

    public double[] getSlipProbability() {
        return slipProbability;
    }

    /**
     * Get the first car type in input file
     *
     * @return first car type in input file
     */
    public String getFirstCarType() {
        return carOrder.get(0);
    }

    /**
     * Get the first driver in input file
     *
     * @return first driver in input file
     */
    public String getFirstDriver() {
        return driverOrder.get(0);
    }

    /**
     * Get the first tire model in input file
     *
     * @return first tire model in input file
     */
    public Tire getFirstTireModel() {
        return tireOrder.get(0);
    }

    /**
     * Return the index of car in terms of order in which is appeared in input
     * file. This index can be used to access fuel and slip probability
     * matrices.
     *
     * @param car the car type
     * @return index of car type as it appeared in input
     */
    public int getCarIndex(String car) {
        int index = carOrder.indexOf(car);
        if (index == -1) {
            throw new IllegalArgumentException("Invalid car type: " + car);
        }
        return index;
    }

    /**
     * Return the index of driver in terms of order in which is appeared in
     * input file.
     *
     * @param driver the driver
     * @return index of driver type as it appeared in input
     */
    public int getDriverIndex(String driver) {
        int index = driverOrder.indexOf(driver);
        if (index == -1) {
            throw new IllegalArgumentException("Invalid driver type: " + driver);
        }
        return index;
    }

    /**
     * Return the index of terrain in terms of order in which is appeared in
     * input file.
     *
     * @param terrain the terrain
     * @return index of terrain as it appeared in input
     */
    public int getTerrainIndex(Terrain terrain) {
        int index = terrainOrder.indexOf(terrain);
        if (index == -1) {
            throw new IllegalArgumentException("Invalid terrain: " + terrain);
        }
        return index;
    }

    /**
     * Return the index of tire Model in terms of order in which is appeared in
     * input file.
     *
     * @param tire the tire model
     * @return index of tire model as it appeared in input
     */
    public int getTireIndex(Tire tire) {
        int index = tireOrder.indexOf(tire);
        if (index == -1) {
            throw new IllegalArgumentException("Invalid tire model: " + tire);
        }
        return index;
    }

    /**
     * Return the index of a given move from [-4, 5] U {slip, breakdown} in the
     * list of move probabilities. (i.e. -4 = 0, -3 = 1, ..., ProblemSpec.SLIP = 10,
     * ProblemSpec.BREAKDOWN = 11)
     *
     * @param move the move from (-4 to 7, where 6 = slip, 7 = breakdown)
     * @return the index (from 0 to 11)
     */
    public int getIndexOfMove(int move) {
        return move + (-CAR_MIN_MOVE);
    }

    /**
     * Return the move distance from [-4, 5] U {slip, breakdown} given an index in the
     * list of probabilities. i.e. index 0 = -4, 1 = -3, 10 = ProblemSpec.SLIP,
     * 11 = ProblemSpec.BREAKDOWN
     *
     * @param index the index (from 0 to 11)
     * @return the move from (-4 to 7, where 6 = slip, 7 = breakdown)
     */
    public int convertIndexIntoMove(int index) {
        return index - (-CAR_MIN_MOVE);
    }


}
