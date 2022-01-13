#include <geographic_to_cartesian.h>
#include <iostream>
#include <fstream>

// Comment how the input file has to be structured
// Comment how the arguments work

int main (int argc, char **argv){
    std::ifstream input_data;
    std::ofstream output_data;
    std::vector<geographic_msgs::GeoPoint> geo_points_vector; 
    geographic_msgs::GeoPoint geo_point, geo_origin_point;
    geometry_msgs::Point32 xyz_point;

    // Reading file route from argument
    input_data.open(argv[1]);

    // Checking if the file could not be opened
    if(!input_data) { 
        std::cerr << "Error: file could not be opened" << std::endl;
        return 1;
    }

    while ( !input_data.eof() ) { 
        input_data >> geo_point.latitude >> geo_point.longitude >> geo_point.altitude;
        geo_points_vector.push_back(geo_point);
    }
    // Get the origin geo point
    geo_origin_point = geo_points_vector.back();

    // Delete the origin from the vector
    geo_points_vector.pop_back();

    input_data.close();

    // Transforming geometry data to cartesian coordinates and writing in the output file
    output_data.open(argv[2], std::ios::trunc);

    for(auto& aux_geo_point : geo_points_vector){
        xyz_point = geographic_to_cartesian(aux_geo_point, geo_origin_point);
        output_data << xyz_point.x << " " << xyz_point.y << " " << xyz_point.z << std::endl;
    }
    
    output_data.close();

    return 0;
}