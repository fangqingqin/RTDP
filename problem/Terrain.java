package problem;

/** The set of different possible terrain types **/
public enum Terrain {
    DIRT,
    ASPHALT,
    DIRT_STRAIGHT,
    DIRT_SLALOM,
    ASPHALT_STRAIGHT,
    ASPHALT_SLALOM,
    DIRT_STRAIGHT_HILLY,
    DIRT_STRAIGHT_FLAT,
    DIRT_SLALOM_HILLY,
    DIRT_SLALOM_FLAT,
    ASPHALT_STRAIGHT_HILLY,
    ASPHALT_STRAIGHT_FLAT,
    ASPHALT_SLALOM_HILLY,
    ASPHALT_SLALOM_FLAT;

    private String text;

    static {
        DIRT.text = "dirt";
        ASPHALT.text = "asphalt";
        DIRT_STRAIGHT.text = "dirt-straight";
        DIRT_SLALOM.text = "dirt-slalom";
        ASPHALT_STRAIGHT.text = "asphalt-straight";
        ASPHALT_SLALOM.text = "asphalt-slalom";
        DIRT_STRAIGHT_HILLY.text = "dirt-straight-hilly";
        DIRT_STRAIGHT_FLAT.text = "dirt-straight-flat";
        DIRT_SLALOM_HILLY.text = "dirt-slalom-hilly";
        DIRT_SLALOM_FLAT.text = "dirt-slalom-flat";
        ASPHALT_STRAIGHT_HILLY.text = "asphalt-straight-hilly";
        ASPHALT_STRAIGHT_FLAT.text = "asphalt-straight-flat";
        ASPHALT_SLALOM_HILLY.text = "asphalt-slalom-hilly";
        ASPHALT_SLALOM_FLAT.text = "asphalt-slalom-flat";
    }

    public String asString() {
        return text;
    }
}
