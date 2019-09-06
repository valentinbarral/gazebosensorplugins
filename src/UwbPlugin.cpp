/*
MIT License

Copyright (c) 2018 Group of Electronic Technology and Communications. University of A Coruña.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Collision.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/common/common.hh>
#include <gazebo/sensors/Noise.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gtec_msgs/Ranging.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo/rendering/DynamicLines.hh>
#include <tf/transform_datatypes.h>

namespace gazebo
{
    class UwbPlugin : public ModelPlugin
    {

        double rangingStd[151][3] =
        {
            {16.941, 100.29, 35.073},
            {16.732, 99.253, 35.523},
            {16.501, 98.137, 36.002},
            {16.248, 96.938, 36.511},
            {15.969, 95.648, 37.049},
            {15.665, 94.263, 37.615},
            {15.333, 92.776, 38.208},
            {14.973, 91.18, 38.827},
            {14.584, 89.469, 39.468},
            {14.168, 87.637, 40.127},
            {13.724, 85.677, 40.802},
            {13.254, 83.584, 41.485},
            {12.761, 81.353, 42.171},
            {12.248, 78.981, 42.851},
            {11.718, 76.465, 43.518},
            {11.177, 73.807, 44.161},
            {10.628, 71.008, 44.769},
            {10.076, 68.076, 45.33},
            {9.5231, 65.02, 45.831},
            {8.9719, 61.854, 46.256},
            {8.4214, 58.595, 46.588},
            {7.8677, 55.265, 46.807},
            {7.3026, 51.888, 46.89},
            {6.712, 48.49, 46.811},
            {6.0741, 45.097, 46.54},
            {5.3571, 41.732, 46.044},
            {4.5155, 38.415, 45.29},
            {3.4851, 35.159, 44.254},
            {2.1755, 31.97, 42.932},
            {0.46033, 28.849, 41.346},
            {1.8309, 25.796, 39.554},
            {4.8956, 22.83, 37.639},
            {8.8609, 20.017, 35.72},
            {13.567, 17.515, 33.945},
            {18.411, 15.656, 32.46},
            {22.593, 15.011, 31.342},
            {25.613, 16.405, 30.581},
            {27.42, 20.748, 30.103},
            {28.18, 28.621, 29.814},
            {28.002, 39.789, 29.625},
            {26.802, 53.048, 29.444},
            {24.293, 66.665, 29.123},
            {20.15, 79.121, 28.402},
            {14.541, 89.642, 27.048},
            {8.653, 98.294, 25.334},
            {4.0812, 105.86, 23.902},
            {1.4846, 113.73, 22.706},
            {0.59708, 124.02, 21.391},
            {1.2152, 139.83, 21.211},
            {3.7417, 165.08, 23.182},
            {9.0128, 202.33, 24.742},
            {16.518, 247.71, 24.415},
            {22.625, 290.08, 21.612},
            {25.341, 320.76, 16.905},
            {26.123, 339.08, 14.264},
            {26.22, 348.04, 14.347},
            {26.097, 350.4, 14.814},
            {26.267, 347.23, 15.46},
            {27.456, 338, 17.818},
            {28.763, 320.25, 22.772},
            {29.041, 293.07, 26.042},
            {28.235, 267, 24.737},
            {25.751, 249.99, 20.399},
            {20.664, 238.53, 17.711},
            {14.472, 226.29, 17.19},
            {10.821, 210.62, 17.187},
            {9.653, 196.12, 17.262},
            {9.531, 187.97, 17.421},
            {10.063, 186.46, 17.792},
            {11.719, 191.35, 18.567},
            {14.929, 203.78, 19.804},
            {16.398, 218.63, 21.194},
            {14.488, 218.37, 22.347},
            {12.583, 202.95, 23.136},
            {12.022, 193.26, 23.643},
            {12.959, 198.65, 24.015},
            {15.791, 217.99, 24.391},
            {20.674, 234.04, 24.91},
            {26.228, 212.04, 25.748},
            {30.554, 155.29, 27.151},
            {33.208, 108.53, 29.408},
            {34.684, 83.004, 32.707},
            {35.487, 69.013, 36.93},
            {35.929, 59.637, 41.621},
            {36.168, 54.556, 46.214},
            {36.261, 59.47, 50.292},
            {36.208, 78.323, 53.675},
            {35.98, 104.28, 56.359},
            {35.569, 126.76, 58.431},
            {35.012, 141.79, 60},
            {34.385, 150.53, 61.177},
            {33.759, 155.11, 62.051},
            {33.18, 157.19, 62.698},
            {32.663, 158.01, 63.175},
            {32.198, 158.71, 63.53},
            {31.767, 160.61, 63.796},
            {31.349, 164.91, 64.001},
            {30.924, 172.17, 64.165},
            {30.477, 181.86, 64.304},
            {29.995, 192.66, 64.427},
            {29.469, 203.22, 64.543},
            {28.894, 212.66, 64.654},
            {28.267, 220.7, 64.765},
            {27.587, 227.39, 64.875},
            {26.858, 232.97, 64.986},
            {26.082, 237.73, 65.097},
            {25.267, 241.91, 65.207},
            {24.419, 245.71, 65.316},
            {23.547, 249.31, 65.423},
            {22.659, 252.8, 65.527},
            {21.764, 256.27, 65.628},
            {20.871, 259.76, 65.725},
            {19.988, 263.32, 65.818},
            {19.121, 266.94, 65.906},
            {18.277, 270.63, 65.99},
            {17.462, 274.38, 66.069},
            {16.678, 278.16, 66.143},
            {15.93, 281.96, 66.213},
            {15.219, 285.75, 66.278},
            {14.546, 289.52, 66.339},
            {13.912, 293.23, 66.395},
            {13.316, 296.88, 66.448},
            {12.759, 300.43, 66.497},
            {12.238, 303.88, 66.542},
            {11.752, 307.21, 66.584},
            {11.3, 310.42, 66.623},
            {10.88, 313.49, 66.658},
            {10.491, 316.43, 66.691},
            {10.13, 319.23, 66.722},
            {9.7953, 321.89, 66.75},
            {9.4858, 324.41, 66.775},
            {9.1994, 326.79, 66.799},
            {8.9345, 329.04, 66.821},
            {8.6896, 331.16, 66.841},
            {8.4632, 333.16, 66.859},
            {8.254, 335.03, 66.876},
            {8.0606, 336.79, 66.892},
            {7.8819, 338.44, 66.906},
            {7.7167, 339.99, 66.919},
            {7.5639, 341.43, 66.931},
            {7.4227, 342.79, 66.942},
            {7.2922, 344.05, 66.952},
            {7.1715, 345.23, 66.961},
            {7.0598, 346.34, 66.969},
            {6.9566, 347.37, 66.977},
            {6.861, 348.33, 66.984},
            {6.7726, 349.22, 66.991},
            {6.6908, 350.06, 66.997},
            {6.615, 350.84, 67.002},
            {6.5449, 351.57, 67.007},
            {6.48, 352.24, 67.012}
        };


        double rssMean[151][3] =
        {
            {-81.695, -78.867, -86.696},
            {-81.639, -78.937, -86.661},
            {-81.579, -79.011, -86.623},
            {-81.515, -79.09, -86.583},
            {-81.446, -79.173, -86.539},
            {-81.373, -79.26, -86.491},
            {-81.295, -79.352, -86.441},
            {-81.213, -79.449, -86.386},
            {-81.127, -79.55, -86.328},
            {-81.036, -79.655, -86.267},
            {-80.943, -79.765, -86.203},
            {-80.847, -79.878, -86.136},
            {-80.749, -79.996, -86.067},
            {-80.65, -80.117, -85.996},
            {-80.552, -80.242, -85.925},
            {-80.455, -80.369, -85.854},
            {-80.361, -80.498, -85.785},
            {-80.273, -80.629, -85.72},
            {-80.191, -80.761, -85.659},
            {-80.118, -80.894, -85.606},
            {-80.055, -81.027, -85.563},
            {-80.005, -81.159, -85.531},
            {-79.97, -81.29, -85.513},
            {-79.954, -81.42, -85.514},
            {-79.96, -81.549, -85.536},
            {-79.993, -81.677, -85.583},
            {-80.062, -81.806, -85.661},
            {-80.176, -81.936, -85.773},
            {-80.351, -82.072, -85.92},
            {-80.606, -82.217, -86.098},
            {-80.961, -82.377, -86.299},
            {-81.423, -82.562, -86.504},
            {-81.964, -82.782, -86.696},
            {-82.51, -83.054, -86.86},
            {-82.973, -83.398, -86.988},
            {-83.305, -83.833, -87.08},
            {-83.516, -84.375, -87.143},
            {-83.642, -85.018, -87.19},
            {-83.714, -85.726, -87.234},
            {-83.755, -86.425, -87.296},
            {-83.779, -87.034, -87.411},
            {-83.798, -87.502, -87.643},
            {-83.828, -87.827, -88.072},
            {-83.905, -88.042, -88.687},
            {-84.074, -88.189, -89.267},
            {-84.334, -88.302, -89.603},
            {-84.6, -88.415, -89.689},
            {-84.785, -88.561, -89.509},
            {-84.867, -88.781, -88.973},
            {-84.868, -89.115, -88.296},
            {-84.849, -89.567, -87.903},
            {-84.946, -90.052, -87.868},
            {-85.204, -90.441, -88.183},
            {-85.44, -90.69, -88.799},
            {-85.567, -90.803, -89.371},
            {-85.636, -90.751, -89.697},
            {-85.709, -90.463, -89.8},
            {-85.819, -89.858, -89.72},
            {-85.898, -89.059, -89.313},
            {-85.821, -88.599, -88.521},
            {-85.683, -88.728, -88.048},
            {-85.645, -89.049, -88.4},
            {-85.761, -89.315, -89.402},
            {-86.028, -89.585, -90.23},
            {-86.367, -89.981, -90.598},
            {-86.66, -90.493, -90.737},
            {-86.833, -90.922, -90.802},
            {-86.881, -91.175, -90.863},
            {-86.825, -91.339, -90.967},
            {-86.702, -91.53, -91.144},
            {-86.563, -91.791, -91.381},
            {-86.314, -91.943, -91.614},
            {-85.964, -91.763, -91.789},
            {-85.73, -91.442, -91.902},
            {-85.688, -91.282, -91.975},
            {-85.843, -91.375, -92.035},
            {-86.203, -91.717, -92.102},
            {-86.703, -92.137, -92.197},
            {-87.138, -92.249, -92.34},
            {-87.371, -92.002, -92.551},
            {-87.453, -91.756, -92.85},
            {-87.474, -91.636, -93.239},
            {-87.479, -91.602, -93.695},
            {-87.481, -91.628, -94.175},
            {-87.484, -91.702, -94.634},
            {-87.487, -91.739, -95.046},
            {-87.486, -91.558, -95.4},
            {-87.476, -91.147, -95.704},
            {-87.453, -90.721, -95.965},
            {-87.42, -90.428, -96.197},
            {-87.379, -90.29, -96.407},
            {-87.334, -90.285, -96.603},
            {-87.289, -90.398, -96.788},
            {-87.245, -90.626, -96.967},
            {-87.203, -90.963, -97.139},
            {-87.162, -91.395, -97.305},
            {-87.122, -91.89, -97.466},
            {-87.08, -92.403, -97.619},
            {-87.036, -92.891, -97.766},
            {-86.988, -93.32, -97.904},
            {-86.936, -93.677, -98.035},
            {-86.879, -93.962, -98.156},
            {-86.816, -94.185, -98.269},
            {-86.747, -94.358, -98.374},
            {-86.673, -94.491, -98.47},
            {-86.594, -94.596, -98.559},
            {-86.509, -94.68, -98.64},
            {-86.42, -94.748, -98.714},
            {-86.327, -94.806, -98.781},
            {-86.23, -94.856, -98.843},
            {-86.132, -94.901, -98.898},
            {-86.032, -94.942, -98.949},
            {-85.931, -94.98, -98.996},
            {-85.83, -95.015, -99.038},
            {-85.73, -95.049, -99.076},
            {-85.631, -95.081, -99.11},
            {-85.535, -95.112, -99.142},
            {-85.441, -95.142, -99.171},
            {-85.349, -95.17, -99.197},
            {-85.261, -95.197, -99.22},
            {-85.176, -95.222, -99.242},
            {-85.095, -95.246, -99.261},
            {-85.017, -95.268, -99.279},
            {-84.943, -95.29, -99.296},
            {-84.873, -95.309, -99.31},
            {-84.806, -95.328, -99.324},
            {-84.743, -95.345, -99.336},
            {-84.684, -95.361, -99.347},
            {-84.628, -95.375, -99.357},
            {-84.575, -95.389, -99.367},
            {-84.525, -95.401, -99.375},
            {-84.479, -95.413, -99.383},
            {-84.435, -95.424, -99.39},
            {-84.395, -95.434, -99.396},
            {-84.356, -95.443, -99.402},
            {-84.321, -95.451, -99.408},
            {-84.287, -95.459, -99.413},
            {-84.256, -95.466, -99.417},
            {-84.227, -95.472, -99.421},
            {-84.2, -95.478, -99.425},
            {-84.175, -95.484, -99.428},
            {-84.151, -95.489, -99.431},
            {-84.129, -95.493, -99.434},
            {-84.108, -95.498, -99.437},
            {-84.089, -95.502, -99.439},
            {-84.072, -95.505, -99.441},
            {-84.055, -95.509, -99.443},
            {-84.04, -95.512, -99.445},
            {-84.025, -95.515, -99.447},
            {-84.012, -95.517, -99.448},
            {-83.999, -95.52, -99.45}
        };

        double rssStd[151][3] =
        {
            {0.5536, 1.4887, 1.0451},
            {0.53015, 1.4665, 1.0389},
            {0.50449, 1.4428, 1.0323},
            {0.47648, 1.4176, 1.0252},
            {0.44599, 1.3909, 1.0176},
            {0.41292, 1.3626, 1.0096},
            {0.3772, 1.3326, 1.0012},
            {0.3388, 1.3009, 0.99228},
            {0.29776, 1.2675, 0.98299},
            {0.25419, 1.2324, 0.97333},
            {0.20828, 1.1955, 0.96337},
            {0.16034, 1.1569, 0.95317},
            {0.11075, 1.1166, 0.94281},
            {0.060033, 1.0747, 0.93236},
            {0.0088092, 1.0312, 0.92191},
            {0.0422, 0.98639, 0.91149},
            {0.092182, 0.94036, 0.90115},
            {0.14026, 0.89334, 0.89087},
            {0.1855, 0.84559, 0.88054},
            {0.22695, 0.79742, 0.87},
            {0.26363, 0.7492, 0.85896},
            {0.29454, 0.70134, 0.84697},
            {0.31862, 0.65426, 0.83345},
            {0.33473, 0.60845, 0.8176},
            {0.34156, 0.56437, 0.79845},
            {0.33755, 0.52252, 0.77488},
            {0.3208, 0.48333, 0.74574},
            {0.2891, 0.44724, 0.71022},
            {0.24029, 0.41461, 0.66844},
            {0.17359, 0.38578, 0.62216},
            {0.09289, 0.36108, 0.57531},
            {0.013005, 0.34086, 0.53326},
            {0.035512, 0.3256, 0.50063},
            {0.017056, 0.31605, 0.47901},
            {0.077979, 0.31331, 0.46664},
            {0.21801, 0.31889, 0.46023},
            {0.35735, 0.33431, 0.45685},
            {0.46686, 0.36022, 0.45448},
            {0.53755, 0.39518, 0.45165},
            {0.56993, 0.43511, 0.44666},
            {0.56473, 0.47443, 0.43672},
            {0.51879, 0.50813, 0.41771},
            {0.42755, 0.53319, 0.38738},
            {0.29814, 0.54867, 0.35597},
            {0.16617, 0.55469, 0.34185},
            {0.080422, 0.55089, 0.33219},
            {0.054798, 0.53508, 0.30204},
            {0.073481, 0.50219, 0.26479},
            {0.13596, 0.44437, 0.28104},
            {0.26916, 0.35586, 0.3419},
            {0.49613, 0.25003, 0.37958},
            {0.7143, 0.18207, 0.37536},
            {0.68584, 0.22135, 0.32594},
            {0.47464, 0.36005, 0.25329},
            {0.30788, 0.51195, 0.23375},
            {0.22249, 0.61231, 0.26297},
            {0.18347, 0.65142, 0.29435},
            {0.19232, 0.64681, 0.34093},
            {0.30655, 0.61761, 0.44295},
            {0.4916, 0.53589, 0.60325},
            {0.62086, 0.37014, 0.70186},
            {0.65469, 0.20637, 0.66507},
            {0.60356, 0.11866, 0.53997},
            {0.50679, 0.10125, 0.45964},
            {0.41269, 0.11749, 0.44066},
            {0.32031, 0.11347, 0.43821},
            {0.25904, 0.077895, 0.4393},
            {0.25106, 0.050947, 0.4433},
            {0.29404, 0.053479, 0.45278},
            {0.36202, 0.097274, 0.47113},
            {0.39112, 0.2084, 0.49682},
            {0.38035, 0.41087, 0.52011},
            {0.39703, 0.68076, 0.53421},
            {0.42078, 0.92069, 0.54075},
            {0.40182, 1.0424, 0.54311},
            {0.31998, 1.041, 0.54311},
            {0.18632, 0.9336, 0.54133},
            {0.070274, 0.75806, 0.53785},
            {0.062407, 0.57303, 0.5329},
            {0.15575, 0.41223, 0.52774},
            {0.26997, 0.30661, 0.52539},
            {0.35698, 0.24765, 0.52976},
            {0.41158, 0.20766, 0.54288},
            {0.44292, 0.16753, 0.56287},
            {0.46011, 0.12565, 0.58535},
            {0.46955, 0.1131, 0.60641},
            {0.47563, 0.17804, 0.62398},
            {0.48157, 0.30799, 0.63759},
            {0.48953, 0.4354, 0.64761},
            {0.50058, 0.52176, 0.6547},
            {0.51463, 0.56575, 0.65953},
            {0.53062, 0.57601, 0.66269},
            {0.547, 0.55897, 0.66467},
            {0.56218, 0.51812, 0.66583},
            {0.57506, 0.45684, 0.66647},
            {0.58505, 0.38115, 0.66681},
            {0.592, 0.30027, 0.66701},
            {0.59603, 0.22405, 0.66715},
            {0.59735, 0.15936, 0.66731},
            {0.59622, 0.10852, 0.66752},
            {0.59286, 0.070518, 0.66778},
            {0.58747, 0.042917, 0.66809},
            {0.58025, 0.023247, 0.66844},
            {0.57137, 0.0094803, 0.66881},
            {0.561, 0.00010983, 0.6692},
            {0.54933, 0.0059465, 0.66958},
            {0.53658, 0.0094526, 0.66995},
            {0.52295, 0.010947, 0.6703},
            {0.5087, 0.010814, 0.67062},
            {0.49406, 0.009331, 0.67092},
            {0.47928, 0.0067051, 0.67118},
            {0.46459, 0.0030986, 0.67142},
            {0.45022, 0.001356, 0.67164},
            {0.43637, 0.0065443, 0.67182},
            {0.42318, 0.012363, 0.67198},
            {0.4108, 0.018716, 0.67212},
            {0.39931, 0.025511, 0.67225},
            {0.38878, 0.032662, 0.67235},
            {0.37923, 0.040086, 0.67244},
            {0.37068, 0.047704, 0.67252},
            {0.36309, 0.055442, 0.67258},
            {0.35643, 0.063235, 0.67264},
            {0.35065, 0.07102, 0.67268},
            {0.34568, 0.078743, 0.67272},
            {0.34147, 0.086357, 0.67276},
            {0.33794, 0.093822, 0.67278},
            {0.33503, 0.1011, 0.67281},
            {0.33266, 0.10817, 0.67283},
            {0.33078, 0.11501, 0.67284},
            {0.32932, 0.1216, 0.67286},
            {0.32822, 0.12793, 0.67287},
            {0.32744, 0.13399, 0.67288},
            {0.32692, 0.13977, 0.67289},
            {0.32663, 0.14529, 0.6729},
            {0.32653, 0.15053, 0.6729},
            {0.32659, 0.1555, 0.67291},
            {0.32678, 0.1602, 0.67291},
            {0.32707, 0.16466, 0.67291},
            {0.32744, 0.16886, 0.67292},
            {0.32788, 0.17282, 0.67292},
            {0.32836, 0.17655, 0.67292},
            {0.32888, 0.18006, 0.67292},
            {0.32942, 0.18336, 0.67292},
            {0.32998, 0.18646, 0.67292},
            {0.33054, 0.18937, 0.67293},
            {0.3311, 0.19209, 0.67293},
            {0.33166, 0.19465, 0.67293},
            {0.33221, 0.19704, 0.67293},
            {0.33274, 0.19927, 0.67293},
            {0.33326, 0.20137, 0.67293},
            {0.33376, 0.20332, 0.67293}
        };

        enum LOSType
        {
            LOS,
            NLOS,
            NLOS_S,
            NLOS_H
        };

    public:
        UwbPlugin() :
            ModelPlugin(),
            sequence(0)
        {
            this->updatePeriod = common::Time(0.0);
        }

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            if (!_sdf->HasElement("update_rate"))
            {
                ROS_FATAL_STREAM("GTEC UWB Plugin needs the parameter: update_rate");
            }

            this->model = _parent;
            this->world = _parent->GetWorld();
            this->SetUpdateRate(_sdf->Get<double>("update_rate"));
            this->nlosSoftWallWidth = 0.25;
            this->tagZOffset = 0;
            this->tagId = 0;
            this->maxDBDistance = 15;
            this->stepDBDistance = 0.1;
            this->allBeaconsAreLOS = false;
            this->useParentAsReference = false;


            if (_sdf->HasElement("all_los"))
            {
                this->allBeaconsAreLOS = _sdf->Get<double>("all_los");
            }

            if (_sdf->HasElement("tag_id"))
            {
                this->tagId = _sdf->Get<double>("tag_id");
            }

            if (_sdf->HasElement("tag_z_offset"))
            {
                this->tagZOffset = _sdf->Get<double>("tag_z_offset");
            }

            if (_sdf->HasElement("nlosSoftWallWidth"))
            {
                this->nlosSoftWallWidth = _sdf->Get<double>("nlosSoftWallWidth");
            }

            if (_sdf->HasElement("tag_link"))
            {
                std::string tag_link = _sdf->Get<std::string>("tag_link");
                this->tagLink = _parent->GetLink(tag_link);

                ROS_INFO("Parent name: %s ChildCount: %d", _parent->GetName().c_str(), _parent->GetChildCount());
                if (this->tagLink == NULL)
                {
                    std::vector<physics::LinkPtr> links = _parent->GetLinks();
                    for (int i = 0; i < links.size(); ++i)
                    {
                        ROS_INFO("Link[%d]: %s", i, links[i]->GetName().c_str());
                    }
                    ROS_INFO("UWB Plugin Tag link Is NULL We use The Parent As Reference");
                    this->useParentAsReference = true;
                }
            }

            if (_sdf->HasElement("anchor_prefix"))
            {
                this->anchorPrefix = _sdf->Get<std::string>("anchor_prefix");
            }
            else
            {
                this->anchorPrefix = "uwb_anchor";
            }

            ROS_INFO("GTEC UWB Plugin is running. Tag %d", this->tagId);
            ROS_INFO("GTEC UWB Plugin All parameters loaded");

            this->lastUpdateTime = common::Time(0.0);

            std::ostringstream stringStream;
            stringStream << "/gtec/gazebo/uwb/ranging/" << this->tagId;
            std::string topicRanging = stringStream.str();

            ROS_INFO("GTEC UWB Plugin Ranging Publishing in %s", topicRanging.c_str());

            stringStream.str("");
            stringStream.clear();
            stringStream << "/gtec/gazebo/uwb/anchors/" << this->tagId;
            std::string topicAnchors = stringStream.str();

            ROS_INFO("GTEC UWB Plugin Anchors Position Publishing in %s", topicAnchors.c_str());

            ros::NodeHandle n;
            this->gtecUwbPub = n.advertise<gtec_msgs::Ranging>(topicRanging, 1000);
            this->gtecAnchors = n.advertise<visualization_msgs::MarkerArray>(topicAnchors, 1000);

            this->firstRay = boost::dynamic_pointer_cast<physics::RayShape>(
                                 this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));

            this->secondRay = boost::dynamic_pointer_cast<physics::RayShape>(
                                  this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));

            this->updateConnection =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&UwbPlugin::OnUpdate, this, _1));
        }

    public:
        void OnUpdate(const common::UpdateInfo &_info)
        {
            common::Time simTime = _info.simTime;
            common::Time elapsed = simTime - this->lastUpdateTime;
            if (elapsed >= this->updatePeriod)
            {
                this->lastUpdateTime = _info.simTime;


                ignition::math::Pose3d tagPose;

                if (!this->useParentAsReference)
                {
                    tagPose = this->tagLink->WorldPose();
                }
                else
                {
                    tagPose = this->model->WorldPose();
                }

                ignition::math::Vector3d posCorrectedZ(tagPose.Pos().X(), tagPose.Pos().Y(), tagPose.Pos().Z() + this->tagZOffset);
                tagPose.Set(posCorrectedZ, tagPose.Rot());
                ignition::math::Vector3d currentTagPose(tagPose.Pos());

                tf::Quaternion q(tagPose.Rot().X(),
                                 tagPose.Rot().Y(),
                                 tagPose.Rot().Z(),
                                 tagPose.Rot().W());

                tf::Matrix3x3 m(q);
                double roll, pitch, currentYaw;
                m.getRPY(roll, pitch, currentYaw);

                // if (currentYaw < 0)
                // {
                //     currentYaw = 2 * M_PI + currentYaw;
                // }

                double startAngle = currentYaw;
                double currentAngle = 0;
                double arc = 3 * M_PI / 2;
                int numAnglesToTestBySide = 30;
                double incrementAngle = arc / numAnglesToTestBySide;
                int totalNumberAnglesToTest = 1 + 2 * numAnglesToTestBySide;
                double anglesToTest[totalNumberAnglesToTest];

                anglesToTest[0] = startAngle;
                for (int i = 1; i < totalNumberAnglesToTest; ++i)
                {
                    double angleToTest;
                    if (i % 2 == 0)
                    {
                        angleToTest = startAngle - (i / 2) * incrementAngle;
                        // if (angleToTest < 0)
                        // {
                        //     angleToTest = 2 * M_PI + angleToTest;
                        // }
                    }
                    else
                    {
                        angleToTest = startAngle + (i - (i - 1) / 2) * incrementAngle;
                        // if (angleToTest > 2 * M_PI)
                        // {
                        //     angleToTest = angleToTest - 2 * M_PI;
                        // }
                    }
                    anglesToTest[i] = angleToTest;
                }

                visualization_msgs::MarkerArray markerArray;
                visualization_msgs::MarkerArray interferencesArray;

                physics::Model_V models = this->world->Models();
                for (physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)
                {

                    if ((*iter)->GetName().find(this->anchorPrefix) == 0)
                    {
                        physics::ModelPtr anchor = *iter;
                        std::string aidStr = anchor->GetName().substr(this->anchorPrefix.length());
                        int aid = std::stoi(aidStr);
                        ignition::math::Pose3d anchorPose = anchor->WorldPose();

                        LOSType losType = LOS;
                        double distance = tagPose.Pos().Distance(anchorPose.Pos());
                        double distanceAfterRebounds = 0;

                        if (!allBeaconsAreLOS)
                        {
                            //We check if a ray can reach the anchor:
                            double distanceToObstacleFromTag;
                            std::string obstacleName;

                            ignition::math::Vector3d directionToAnchor = (anchorPose.Pos() - tagPose.Pos()).Normalize();
                            this->firstRay->Reset();
                            this->firstRay->SetPoints(tagPose.Pos(), anchorPose.Pos());
                            this->firstRay->GetIntersection(distanceToObstacleFromTag, obstacleName);

                            if (obstacleName.compare("") == 0)
                            {
                                //There is no obstacle between anchor and tag, we use the LOS model
                                losType = LOS;
                                distanceAfterRebounds = distance;
                            }
                            else
                            {

                                //We use a second ray to measure the distance from anchor to tag, so we can
                                //know what is the width of the walls
                                double distanceToObstacleFromAnchor;
                                std::string otherObstacleName;

                                this->secondRay->Reset();
                                this->secondRay->SetPoints(anchorPose.Pos(), tagPose.Pos());
                                this->secondRay->GetIntersection(distanceToObstacleFromAnchor, otherObstacleName);

                                double wallWidth = distance - distanceToObstacleFromTag - distanceToObstacleFromAnchor;

                                if (wallWidth <= this->nlosSoftWallWidth && obstacleName.compare(otherObstacleName) == 0)
                                {
                                    //We use NLOS - SOFT model
                                    losType = NLOS_S;
                                    distanceAfterRebounds = distance;
                                }
                                else
                                {
                                    //We try to find a rebound to reach the anchor from the tag
                                    bool end = false;

                                    double maxDistance = 30;
                                    double distanceToRebound = 0;
                                    double distanceToFinalObstacle = 0;
                                    double distanceNlosHard = 0;

                                    std::string finalObstacleName;
                                    int indexRay = 0;
                                    bool foundNlosH = false;
                                    while (!end)
                                    {

                                        currentAngle = anglesToTest[indexRay];

                                        double x = currentTagPose.X() + maxDistance * cos(currentAngle);
                                        double y = currentTagPose.Y() + maxDistance * sin(currentAngle);

                                        ignition::math::Vector3d rayPoint(x, y, currentTagPose.Z());

                                        this->firstRay->Reset();
                                        this->firstRay->SetPoints(currentTagPose, rayPoint);
                                        this->firstRay->GetIntersection(distanceToRebound, obstacleName);

                                        if (obstacleName.compare("") != 0)
                                        {
                                            ignition::math::Vector3d collisionPoint(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), currentTagPose.Z());

                                            //We try to reach the anchor from here
                                            this->secondRay->Reset();
                                            this->secondRay->SetPoints(collisionPoint, anchorPose.Pos());
                                            this->secondRay->GetIntersection(distanceToFinalObstacle, finalObstacleName);

                                            if (finalObstacleName.compare("") == 0)
                                            {
                                                //We reach the anchor after one rebound
                                                distanceToFinalObstacle = anchorPose.Pos().Distance(collisionPoint);

                                                if (distanceToRebound + distanceToFinalObstacle <= maxDBDistance)
                                                {
                                                    foundNlosH = true;
                                                    //We try to find the shortest rebound
                                                    if (distanceNlosHard < 0.1)
                                                    {
                                                        distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                    }
                                                    else if (distanceNlosHard > distanceToRebound + distanceToFinalObstacle)
                                                    {
                                                        distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                    }
                                                }
                                            }
                                        }

                                        if (indexRay < totalNumberAnglesToTest - 1)
                                        {
                                            indexRay += 1;
                                        }
                                        else
                                        {
                                            end = true;
                                        }
                                    }

                                    if (foundNlosH)
                                    {
                                        //We use the NLOS Hard Model with distance = distanceNlosHard
                                        losType = NLOS_H;
                                        distanceAfterRebounds = distanceNlosHard;
                                    }
                                    else
                                    {
                                        //We can not reach the anchor, no ranging.
                                        losType = NLOS;
                                    }
                                }
                            }

                        }
                        else
                        {
                            //All beacons are LOS
                            losType = LOS;
                            distanceAfterRebounds = distance;
                        }

                        if ((losType == LOS || losType == NLOS_S) && distanceAfterRebounds > maxDBDistance)
                        {
                            losType = NLOS;
                        }

                        if (losType == NLOS_H && distanceAfterRebounds > maxDBDistance)
                        {
                            losType = NLOS;
                        }

                        if (losType != NLOS)
                        {

                            int indexScenario = 0;
                            if (losType == NLOS_S)
                            {
                                indexScenario = 2;
                            }
                            else if (losType == NLOS_H)
                            {
                                indexScenario = 1;
                            }

                            int indexRanging = (int) round(distanceAfterRebounds / stepDBDistance);

                            std::normal_distribution<double> distributionRanging(distanceAfterRebounds * 1000, rangingStd[indexRanging][indexScenario]);
                            std::normal_distribution<double> distributionRss(rssMean[indexRanging][indexScenario], rssStd[indexRanging][indexScenario]);

                            double rangingValue = distributionRanging(this->random_generator);
                            double powerValue = distributionRss(this->random_generator);

                            gtec_msgs::Ranging ranging_msg;
                            ranging_msg.anchorId = aid;
                            ranging_msg.tagId = this->tagId;
                            ranging_msg.range = rangingValue;
                            ranging_msg.seq = this->sequence;
                            ranging_msg.rss = powerValue;
                            ranging_msg.errorEstimation = 0.00393973;
                            this->gtecUwbPub.publish(ranging_msg);
                        }

                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "world";
                        marker.header.stamp = ros::Time();
                        marker.id = aid;
                        marker.type = visualization_msgs::Marker::CYLINDER;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position.x = anchorPose.Pos().X();
                        marker.pose.position.y = anchorPose.Pos().Y();
                        marker.pose.position.z = anchorPose.Pos().Z();
                        marker.pose.orientation.x = anchorPose.Rot().X();
                        marker.pose.orientation.y = anchorPose.Rot().Y();
                        marker.pose.orientation.z = anchorPose.Rot().Z();
                        marker.pose.orientation.w = anchorPose.Rot().W();
                        marker.scale.x = 0.2;
                        marker.scale.y = 0.2;
                        marker.scale.z = 0.5;
                        marker.color.a = 1.0;

                        if (losType == LOS)
                        {
                            marker.color.r = 0.0;
                            marker.color.g = 0.6;
                            marker.color.b = 0.0;
                        }
                        else if (losType == NLOS_S)
                        {
                            marker.color.r = 0.6;
                            marker.color.g = 0.6;
                            marker.color.b = 0.0;
                        }
                        else if (losType == NLOS_H)
                        {
                            marker.color.r = 0.0;
                            marker.color.g = 0.0;
                            marker.color.b = 0.6;
                        }
                        else if (losType == NLOS)
                        {
                            marker.color.r = 0.6;
                            marker.color.g = 0.0;
                            marker.color.b = 0.0;
                        }

                        markerArray.markers.push_back(marker);
                    }
                }

                this->gtecAnchors.publish(markerArray);
                this->sequence++;
            }
        }

    public:
        void SetUpdateRate(double _rate)
        {
            if (_rate > 0.0)
            {
                this->updatePeriod = 1.0 / _rate;
            }
            else
            {
                this->updatePeriod = 0.0;
            }
        }

    public:
        void Reset() override
        {
            ROS_INFO("GTEC UWB Plugin RESET");
            this->lastUpdateTime = common::Time(0.0);
        }

    private:
        physics::ModelPtr model;
    private:
        physics::WorldPtr world;
    private:
        physics::RayShapePtr firstRay;
    private:
        physics::RayShapePtr secondRay;
    private:
        event::ConnectionPtr updateConnection;
    private:
        common::Time updatePeriod;
    private:
        common::Time lastUpdateTime;
    private:
        double tagZOffset;
    private:
        std::string anchorPrefix;
    private:
        physics::LinkPtr tagLink;
    private:
        ros::Publisher gtecUwbPub;
    private:
        ros::Publisher gtecAnchors;
    private:
        unsigned char sequence;
    private:
        double nlosSoftWallWidth;
    private:
        double maxDBDistance;
    private:
        double stepDBDistance;
    private:
        bool allBeaconsAreLOS;
    private:
        int tagId;
    private:
        bool useParentAsReference;
    private:
        std::default_random_engine random_generator;
    };

    GZ_REGISTER_MODEL_PLUGIN(UwbPlugin)
}

