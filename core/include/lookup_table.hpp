#pragma once

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

typedef struct{
    int angle;
    int dist;
}throw_info_t;

// I know it's bad, get over it!
static vector< vector<int> > speeds_table = {
    //dist, speed, angle
    {409,	1450,	1000},
    {637,	1525,	1000},
    {870,	1580,	1000},
    {1160,	1675,	1000},
    {867,	1360,	1500},
    {1164,	1400,	1500},
    {1540,	1460,	1500},
    {1890,	1530,	1500},
    {2170,	1570,	1500},
    {2230,	1575,	1500},
    {2570,	1630,	1500},
    {2890,	1755,	1500},
    {3200,	1885,	1500},
    {3210,	1725,	1700},
    {3490,	1805,	1700},
    {2480,	1575,	1700},
    {2020,	1520,	1700},
    {1540,	1455,	1700},
    {1195,	1395,	1700},
    {1198,	1440,	1352},
    {1548,	1520,	1352},
    {1940,	1585,	1352},
    {2155,	1640,	1352},
    {2635,	1855,	1352},
    {1418,	1480,	1352},
    {1105,	1430,	1352},
    {800,	1370,	1352},
    {567,	1440,	1152},
    {820,	1500,	1152},
    {1537,	1647,	1152},
    {585,	1430,	1152},
    {395,	1355,	1152},
    {1013,	1530,	1152}
};

/**
 * Returns the desired thrower BLDC speed in microseconds for the given
 * distance and angle or -1 if a suitable value cannot be found.
*/
inline int getSpeedForDistAndAngle(int dist, int angle){
    vector< vector<int> > correct_angle;

    for(int i = 0; i < speeds_table.size(); i++){
        if(speeds_table[i][2] == angle){
            correct_angle.push_back(speeds_table[i]);
        }
    }

    int closest_lower  = 0;
    int closest_higher = 9999;
    int closest_higher_spd, closest_lower_spd;

    for(int i = 0; i < correct_angle.size(); i++){
        int elem_dist = correct_angle[i][0];
        int elem_speed = correct_angle[i][1];
        if(dist < elem_dist  &&  closest_higher > elem_dist){
            closest_higher = elem_dist;
            closest_higher_spd = elem_speed;
        }
        if(dist > elem_dist  &&  closest_lower < elem_dist){
            closest_lower = elem_dist;
            closest_lower_spd = elem_speed;
        }

    }

    if(closest_higher == 9999  ||  closest_lower == 0){
        // did not find enough points to get the speed from this angle
        return -1;
    }

    // closest_higher, closest_higher_spd
    // dist, [unknown]
    // closest_lower, closest_lower_spd

    //(dist-closest_lower)/(y-closest_lower_spd) = (closest_higher-closest_lower)/(closest_higher_spd-closest_lower_spd)
    //(dist-closest_lower)*(closest_higher_spd-closest_lower_spd) = (y-closest_lower_spd)*(closest_higher-closest_lower)
    //(dist-closest_lower)*(closest_higher_spd-closest_lower_spd)/(closest_higher-closest_lower) = (y-closest_lower_spd)
    //(dist-closest_lower)*(closest_higher_spd-closest_lower_spd)/(closest_higher-closest_lower)+closest_lower_spd = y
    //y = (dist-closest_lower)*(closest_higher_spd-closest_lower_spd)/(closest_higher-closest_lower)+closest_lower_spd

    int new_speed = (dist-closest_lower)*(closest_higher_spd-closest_lower_spd)/(closest_higher-closest_lower)+closest_lower_spd;
    //cout << "new speed: " << new_speed << endl;
    return new_speed;
}

/**
 * Gets the desired thrower BLDC speed and angle in microseconds for
 * the given distance or -1 for both if the values cannot be found
*/
inline throw_info_t getSpeedForDist(int dist_mm){
    // A wonderful hack made by Jürgen. Remove this once the main logic has been fixed!
    dist_mm -= 100;

    if(dist_mm < 1200){
        return (throw_info_t) { 1152, getSpeedForDistAndAngle(dist_mm, 1152) };
    }else if(dist_mm < 2500){
        return (throw_info_t) { 1352, getSpeedForDistAndAngle(dist_mm, 1352) };
    }else if(dist_mm < 3000){
        return (throw_info_t) { 1500, getSpeedForDistAndAngle(dist_mm, 1500) };
    }else if(dist_mm < 3490){
        return (throw_info_t) { 1700, getSpeedForDistAndAngle(dist_mm, 1700) };
    }else{
        return (throw_info_t) { -1, -1 };
    }
}
