package apollo.comanche055;

import apollo.agc.Agc;

/**
 * STAR_TABLES.agc (Comanche055): unit vectors of the 37 navigation stars, stored in the
 * fixed-memory bank 32 starting at STARTAB. Each component is a double-precision
 * value scaled B-1 (i.e. the stored fraction is the true value / 2); the tables below
 * hold the true unit-vector components. The AGC table is ordered from star 37 down to star 1.
 */
public final class StarTables {
    private final Agc agc;

    public StarTables(Agc agc) {
        this.agc = agc;
    }

    /** STARTAB: rows in AGC order (star 37 first, star 1 last), columns X, Y, Z (true values; AGC scale B-1 means stored = value/2). */
    public static final double[][] STARTAB = {
        {.8342971408, -.2392481515, -.4966976975},  // STAR 37
        {.8139832631, -.5557243189, .1691204557},  // STAR 36
        {.4541086270, -.5392368197, .7092312789},  // STAR 35
        {.3201817378, -.4436021946, -.8370786986},  // STAR 34
        {.5520184464, -.7933187400, -.2567508745},  // STAR 33
        {.4537196908, -.8779508801, .1527766153},  // STAR 32
        {.2069525789, -.8719885748, -.4436288486},  // STAR 31
        {.1217293692, -.7702732847, .6259880410},  // STAR 30
        {-.1124304773, -.9694934200, .2178116072},  // STAR 29
        {-.1146237858, -.3399692557, -.9334250333},  // STAR 28
        {-.3516499609, -.8240752703, -.4441196390},  // STAR 27
        {-.5326876930, -.7160644554, .4511047742},  // STAR 26
        {-.7861763936, -.5217996305, .3311371675},  // STAR 25
        {-.6898393233, -.4182330640, -.5909338474},  // STAR 24
        {-.5812035376, -.2909171294, .7599800468},  // STAR 23
        {-.9170097662, -.3502146628, -.1908999176},  // STAR 22
        {-.4523440203, -.0493710140, -.8904759346},  // STAR 21
        {-.9525211695, -.0593434796, -.2986331746},  // STAR 20
        {-.9656605484, .0525933156, .2544280809},  // STAR 19
        {-.8608205219, .4636213989, .2098647835},  // STAR 18
        {-.7742591356, .6152504197, -.1482892839},  // STAR 17
        {-.4657947941, .4774785033, .7450164351},  // STAR 16
        {-.3612508532, .5747270840, -.7342932655},  // STAR 15
        {-.4118589524, .9065485360, .0924226975},  // STAR 14
        {-.1820751783, .9404899869, -.2869271926},  // STAR 13
        {-.0614937230, .6031563286, -.7952489957},  // STAR 12
        {.1371725575, .6813721061, .7189685267},  // STAR 11
        {.2011399589, .9690337941, -.1432348512},  // STAR 10
        {.3507315038, .8926333307, .2831839492},  // STAR 9
        {.4105636020, .4988110001, .7632988371},  // STAR 8
        {.7032235469, .7075846047, .0692868685},  // STAR 7
        {.5450107404, .5314955466, -.6484410356},  // STAR 6
        {.0130968840, .0078062795, .9998837600},  // STAR 5
        {.4917678276, .2204887125, -.8423473935},  // STAR 4
        {.4775639450, .1166004340, .8708254803},  // STAR 3
        {.9342640400, .1735073142, -.3115219339},  // STAR 2
        {.8748658918, .0260879174, .4836621670},  // STAR 1
    };

    /** CATLOG: catalog identification number (DEC 6970). */
    public static final int CATLOG = 6970;

    /** Returns the unit vector {x,y,z} of star n (1..37). */
    public static double[] star(int n) {
        return STARTAB[37 - n].clone();
    }
}
