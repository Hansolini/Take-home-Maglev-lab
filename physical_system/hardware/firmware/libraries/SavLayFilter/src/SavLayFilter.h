/**
 * Arduino Savitzky Golay Library - Version 1.0.1
 * by James Deromedi <jmderomedi@gmail.com>
 *
 * This Library is licensed under the MIT License
*/

#ifndef SavLayFilter_h
#define SavLayFilter_h
#define LIBRARY_VERSION 1.2.1

class SavLayFilter {

public:
//=======================================================================================
  SavLayFilter(double*, int, int);  //*Constructor. Generic function, calls cubic 
                                    //smoothing table of window size 5
                                    //Links the input to the filter

  float Compute();                  //Calculates the filter specified in the constructor

  void resetValues(){               //Resets entire filter. Be careful when calling
    for (int i = 0; i < 13; i++){   //Since it will ruin current filtering
      _toBeSmoothedArray[i] = 0;
    }
    _fillArrayCount = 0;
    _sum = 0;
    _smoothedValue = 0;
    _arrayPointer = 0;
  };

void UpdateData(double newInput);

//=======================================================================================
private:
  float _calculating(double windowArray[]);   //Internally used to compute
                                              //actually filtered value
  double *myInput;                            //Creates the link to the input, freeing the user from having to specify
  int myWindowSize;                           //Saves the values inputed by the user

  double  (*_convoluteTable)[14];             //Used to internally reference which table to use
  double* _normalizationFactor;               //Used to internally reference which normalization factor

  int _arrayPointer;                          //Pointer for moving through data array
  int _fillArrayCount;                        //Used to check if the data array is filled
  double _sum;                                //Returned value of _calculating()
  double _smoothedValue;                      //Returned value of the filter

  double _toBeSmoothedArray[13];              //Array that will be filled with the inputValues
                                              //Set to thirteen since that is max the window size can go to

/* Contains the convolutes for each window size
 * First column is normization factor*/
    double _quadCubicSmooth[11][14] = {
    { 1,-0.085714,0.342857,0.485714,0.342857,-0.085714,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000},  // Window size 5 
    { 1,-0.095238,0.142857,0.285714,0.333333,0.285714,0.142857,-0.095238,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000},  // Window size 7 
    { 1,0.034965,-0.128205,0.069930,0.314685,0.417249,0.314685,0.069930,-0.128205,0.034965,0.000000,0.000000,0.000000,0.000000},  // Window size 9 
    { 1,0.041958,-0.104895,-0.023310,0.139860,0.279720,0.333333,0.279720,0.139860,-0.023310,-0.104895,0.041958,0.000000,0.000000},  // Window size 11 
    { 1,-0.016671,0.073351,-0.078049,-0.075776,0.098508,0.303102,0.391067,0.303102,0.098508,-0.075776,-0.078049,0.073351,-0.016671},  // Window size 13 
    { 5,-0.020640,0.072239,-0.037787,-0.086132,-0.011114,0.138922,0.277844,0.333333,0.277844,0.138922,-0.011114,-0.086132,-0.037787},  // Window size 15 
    { 4,0.008480,-0.044824,0.070669,0.008480,-0.084803,-0.053708,0.110244,0.296810,0.377305,0.296810,0.110244,-0.053708,-0.084803},  // Window size 17 
    { 3,0.010679,-0.048055,0.054112,0.034863,-0.048055,-0.079149,-0.006596,0.138511,0.277022,0.333333,0.277022,0.138511,-0.006596},  // Window size 19 
    { 2,-0.004469,0.027805,-0.057976,0.023923,0.057282,-0.014721,-0.084145,-0.041579,0.116602,0.292861,0.368832,0.292861,0.116602},  // Window size 21 
    { 5,-0.005687,0.031281,-0.052632,0.002234,0.053558,0.021089,-0.051273,-0.075733,-0.004390,0.138296,0.276591,0.333333,0.276591},  // Window size 23 
    { 0.75,0.002407,-0.017290,0.044800,-0.038066,-0.027686,0.040479,0.045637,-0.026064,-0.082587,-0.033914,0.120583,0.290154,0.363093}  // Window size 25 
    };

    /* Missing window size 17 because of scan line from paper
     * There is no windowsize 5 for this table*/
    double _quarticQuinticSmooth[11][14] = {
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {231,5,-30,75,131,0,0,0,0,0,0,0,0,0},
      {429,15,-55,30,135,179,0,0,0,0,0,0,0,0},
      {429,18,-45,-10,60,120,143,0,0,0,0,0,0,0},
      {2431,110,-198,-160,110,390,600,677,0,0,0,0,0,0},
      {46189,2145,-2860,-2937,-165,3755,7500,10125,11053,0,0,0,0,0},
      {4199,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {7429,340,-255,-420,-290,18,405,790,1110,1320,1393,0,0,0},
      {260015,11628,-6460,-13005,-11220,-3940,6378,17655,28190,36660,42120,44003,0,0},
      {655,285,-114,-285,-285,-165,30,261,495,705,870,975,1011,0},
      {30015,1265,-345,-1122,-1255,-915,-255,590,1503,2385,3155,3750,4125,4253}
    };

    double _quadFirstDerive[11][14] = {
      {10,2,1,0,0,0,0,0,0,0,0,0,0,0},
      {28,3,2,1,0,0,0,0,0,0,0,0,0,0},
      {60,4,3,2,1,0,0,0,0,0,0,0,0,0},
      {110,5,4,3,2,1,0,0,0,0,0,0,0,0},
      {182,6,5,4,3,2,1,0,0,0,0,0,0,0},
      {280,7,6,5,4,3,2,1,0,0,0,0,0,0},
      {408,8,7,6,5,4,3,2,1,0,0,0,0,0},
      {570,9,8,7,6,5,4,3,2,1,0,0,0,0},
      {770,10,9,8,7,6,5,4,3,2,1,0,0,0},
      {1012,11,10,9,8,7,6,5,4,3,2,1,0,0},
      {1300,12,11,10,9,8,7,6,5,4,3,2,1,0}
    };

    double _cubicQuarticFirstDerive[11][14] = {
      {12,1,-8,0,0,0,0,0,0,0,0,0,0,0},
      {252,22,67,-58,0,0,0,0,0,0,0,0,0,0},
      {1188,86,-142,-193,-126,0,0,0,0,0,0,0,0,0},
      {5148,300,-294,-532,-503,-296,0,0,0,0,0,0,0,0},
      {24024,1133,-660,-1578,-1796,-1489,-832,0,0,0,0,0,0,0},
      {334152,12922,-4121,-14150,-18334,-17842,-13843,-7506,0,0,0,0,0,0},
      {23256,748,-98,-643,-930,-1002,-902,-1002,-902,-673,-358,0,0,0},
      {255816,6936,68,-4648,-7481,-8700,-8574,-8179,-5363,-2816,0,0,0,0},
      {3634092,84075,10032,-43284,-78176,-96947,-101900,-95338,-79564,-56881,-29592,0,0,0},
      {197340,3938,815,-1518,-3140,-4130,-4567,-4098,-3350,-2365,-1222,0,0},
      {1776060,30866,8602,-8525,-20982,-29236,-33754,-35003,-33450,-29562,-23806,-16649,-8558,0}
    };

    //There is no line for this table for window size 5
    double _quinticFirstDerive[11][14] = {
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {1,-90,18,-2,0,0,0,0,0,0,0,0,0,0},
      {143,-5758,-4538,2762,-508,0,0,0,0,0,0,0,0,0},
      {143,-3084,-3776,-1244,2166,-573,0,0,0,0,0,0,0,0},
      {2431,-31380,-45741,-33511,-12,27093,-14647,0,0,0,0,0,0,0},
      {20995,-175125,-279975,-266401,-130506,65229,169819,-78351,0,0,0,0,0,0},
      {41990,-23945,-40483,-43973,-32306,-8671,16679,24661,-14404,0,0,0,0,0},
      {81719,-332684,-583549,-686099,-604484,-348823,9473,322378,349928,-255102,0,0,0},
      {5311735,-15977364,-28754154,-35613829,-34807914,-26040033,-10949942,6402438,19052988,16649358,-15033066,0,0,0},
      {312455,-357045,-654687,-840937,-878634,-752859,-478349,-106911,265164,489687,359157,-400653,0,0},
      {7153575,-6356625,-11820675,-15593141,-17062146,-15896511,-12139321,-6301491,544668,6671883,9604353,6024183,-8322182,0}  
    };

    double _quadCubicSecDerive[11][14]= {
      {7,2,-1,-2,0,0,0,0,0,0,0,0,0,0},
      {42,5,0,-3,-4,0,0,0,0,0,0,0,0,0},
      {462,28,7,-8,-17,-20,0,0,0,0,0,0,0,0},
      {429,15,6,-1,-6,-9,-10,0,0,0,0,0,0,0},
      {1001,22,11,2,-5,-10,-13,-14,0,0,0,0,0,0},
      {6188,91,52,19,-8,-29,-48,-53,-56,0,0,0,0,0},
      {3876,40,25,12,1,-8,-15,-20,-23,-24,0,0,0,0},
      {6783,51,34,19,6,-5,-14,-21,-26,-29,-30,0,0},
      {33649,190,133,82,37,-2,-35,-62,-83,-98,-107,-110,0,0},
      {17710,77,56,37,20,5,-8,-19,-28,-35,-40,-43,-44,0},
      {26910,92,69,48,29,12,-3,-16,-27,-36,-43,-48,-51,-52}  
    };

    double _quarticQuinticSecDerive[11][14]= {
      {3,-3,48,-90,0,0,0,0,0,0,0,0,0,0},
      {99,-117,603,-171,-630,0,0,0,0,0,0,0,0,0},
      {4719,-4158,12243,4983,-6963,-12210,0,0,0,0,0,0,0,0},
      {16731,-10530,20358,17082,117,-15912,-22230,0,0,0,0,0,0,0},
      {160446,-72963,98010,115632,53262,-32043,-99528,-124740,0,0,0,0,0,0},
      {277134,-93093,88803,133485,95568,19737,-59253,-116577,-137340,0,0,0,0,0},
      {478686,121524,82251,153387,137085,71592,-11799,-88749,-141873,-160740,0,0,0,0},
      {490314,-96084,45084,105444,109071,76830,26376,-27846,-74601,-105864,-116820,0,0,0},
      {245157,-37791,11628,38802,41412,34353,19734,1878,-15678,-30183,-39672,-42966,0,0},
      {2812095,-346731,61845,281979,358530,331635,236709,104445,-39186,-172935,-280275,-349401,-373230,0},
      {4292145,429594,31119,298155,413409,414786,336201,207579,54855,-100026,-239109,-348429,-418011,-441870}  
    };
    double _cubicQuarticThirdDerive[11][14];
    double _quinticThirdDerive[11][14];
    double _quarticQuinticFourDerive[11][14];
    double _quinticFifthDerive[11][14];

};//END Class

#endif
