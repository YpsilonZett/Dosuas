#pragma once

#include <iostream>
#include <array>
#include <vector>
#include <algorithm>
#include <utility>
#include <stdlib.h>
#include <math.h>

#include <signal.h>
#include <ctime>

#include <SFML/Audio.hpp>
#include <SFML/System.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#define NOMINMAX  // Windows.h causes PCL error, if MINMAX is defined
#include <Windows.h>
#include <MTF_API.h>