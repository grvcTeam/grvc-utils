#include <path_planner.h>

int main(int _argc, char** _argv) {

    geographic_msgs::GeoPoint map_origin_geo;
    map_origin_geo.latitude = 38.138728;  map_origin_geo.longitude = -3.173825;



    std::vector< std::vector<geographic_msgs::GeoPoint> > obstacle_polygon_vector_geo;
    std::vector<geographic_msgs::GeoPoint> geofence_polygon_geo;

    std::vector<geographic_msgs::GeoPoint> current_polygon_geopoint;
    geographic_msgs::GeoPoint current_geopoint;

    current_geopoint.latitude = 38.138621;  current_geopoint.longitude = -3.174005;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.138923;  current_geopoint.longitude = -3.172830;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.138652;  current_geopoint.longitude = -3.172695;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.138347;  current_geopoint.longitude = -3.173893;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.138621;  current_geopoint.longitude = -3.174005;
    current_polygon_geopoint.push_back(current_geopoint);
    obstacle_polygon_vector_geo.push_back(current_polygon_geopoint);
    current_polygon_geopoint.clear();
    current_geopoint.latitude = 38.136774;  current_geopoint.longitude = -3.179633;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.136911;  current_geopoint.longitude = -3.179653;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.137158;  current_geopoint.longitude = -3.180338;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.137120;  current_geopoint.longitude = -3.180940;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.136538;  current_geopoint.longitude = -3.180878;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.136774;  current_geopoint.longitude = -3.179633;
    current_polygon_geopoint.push_back(current_geopoint);
    obstacle_polygon_vector_geo.push_back(current_polygon_geopoint);
    current_polygon_geopoint.clear();
    current_geopoint.latitude = 38.135994;  current_geopoint.longitude = -3.179585;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.134940;  current_geopoint.longitude = -3.179584;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.134502;  current_geopoint.longitude = -3.181708;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.135402;  current_geopoint.longitude = -3.181809;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.136153;  current_geopoint.longitude = -3.180090;
    current_polygon_geopoint.push_back(current_geopoint);
    current_geopoint.latitude = 38.135994;  current_geopoint.longitude = -3.179585;
    current_polygon_geopoint.push_back(current_geopoint);
    obstacle_polygon_vector_geo.push_back(current_polygon_geopoint);
    current_polygon_geopoint.clear();

    current_geopoint.latitude = 38.132439;    current_geopoint.longitude = -3.184625;
    geofence_polygon_geo.push_back(current_geopoint);
    current_geopoint.latitude = 38.143406;    current_geopoint.longitude = -3.182860;
    geofence_polygon_geo.push_back(current_geopoint);
    current_geopoint.latitude = 38.143863;    current_geopoint.longitude = -3.164912;
    geofence_polygon_geo.push_back(current_geopoint);
    current_geopoint.latitude = 38.133770;    current_geopoint.longitude = -3.164140;
    geofence_polygon_geo.push_back(current_geopoint);
    current_geopoint.latitude = 38.132439;    current_geopoint.longitude = -3.184625;
    geofence_polygon_geo.push_back(current_geopoint);



    std::vector<geometry_msgs::Polygon> obstacle_polygon_vector_cartesian;
    geometry_msgs::Polygon geofence_polygon_cartesian;

    for (std::vector<geographic_msgs::GeoPoint> current_polygon_geopoint : obstacle_polygon_vector_geo ) {
        geometry_msgs::Polygon current_polygon_point32;
        for (geographic_msgs::GeoPoint current_geopoint : current_polygon_geopoint ) {
            geometry_msgs::Point32 current_point32 = geographic_to_cartesian(current_geopoint, map_origin_geo);
            current_polygon_point32.points.push_back(current_point32);
        }
        obstacle_polygon_vector_cartesian.push_back(current_polygon_point32);
    }

    for (geographic_msgs::GeoPoint current_geopoint : geofence_polygon_geo ) {
        geometry_msgs::Point32 current_point32 = geographic_to_cartesian(current_geopoint, map_origin_geo);
        geofence_polygon_cartesian.points.push_back(current_point32);
    }



    geographic_msgs::GeoPoint initial_geopoint, final_geopoint;
    initial_geopoint.latitude = 38.132739;  initial_geopoint.longitude = -3.184325; initial_geopoint.altitude = 10;
    final_geopoint.latitude = 38.143663;    final_geopoint.longitude = -3.165112;   final_geopoint.altitude = 20;

    geometry_msgs::PointStamped initial_pointstamped;
    geometry_msgs::PointStamped final_pointstamped;
    geometry_msgs::Point32 initial_point32 = geographic_to_cartesian(initial_geopoint, map_origin_geo);
    geometry_msgs::Point32 final_point32 = geographic_to_cartesian(final_geopoint, map_origin_geo);
    initial_pointstamped.point.x = initial_point32.x;  initial_pointstamped.point.y = initial_point32.y;
    final_pointstamped.point.x = final_point32.x;     final_pointstamped.point.y = final_point32.y;



    grvc::PathPlanner path_planner_geopoint = grvc::PathPlanner(obstacle_polygon_vector_geo, geofence_polygon_geo);
    // std::vector<geographic_msgs::GeoPoint> path_geopoint = path_planner_geopoint.getPath(initial_geopoint, final_geopoint);
    // path_planner_geopoint.getElevations(path_geopoint);
    // std::vector<geographic_msgs::GeoPoint> path_geopoint = path_planner_geopoint.getPathWithAbsoluteAltitude(initial_geopoint, final_geopoint);
    std::vector<geographic_msgs::GeoPoint> path_geopoint = path_planner_geopoint.getPathWithRelativeAltitude(initial_geopoint, final_geopoint, 420.0);


    // grvc::PathPlanner path_planner_pointstamped = grvc::PathPlanner(obstacle_polygon_vector_cartesian, geofence_polygon_cartesian);
    // std::vector<geometry_msgs::PointStamped> path_pointstamped = path_planner_pointstamped.getPath(initial_pointstamped, final_pointstamped);


    return 0;
}