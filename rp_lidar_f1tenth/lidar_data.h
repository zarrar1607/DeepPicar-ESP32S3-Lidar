fndef LIDAR_DATA_H
#define LIDAR_DATA_H

#include <array>

// Declare the array defined in your .cc file
extern const std::array<std::array<float, NumColumns>, NumElements> LidarData;

// Declare the length of the array
extern const int LidarData_length;

#endif // LIDAR_DATA_H

