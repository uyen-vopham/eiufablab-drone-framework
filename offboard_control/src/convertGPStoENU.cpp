#include "offboard_control/convertGPStoENU.hpp"

// using std::placeholders::_1;
ConvertGPStoENU::ConvertGPStoENU (){}

std::string ConvertGPStoENU::convertGPS2ENU_function (double lat0, double long0, double alt0, 
                std::vector<double> line){
    //  double lat0 = 11.052945, lon0 = 106.6661470, alt0 = 0;     // gốc local
    // double lat = 11.0529021, lon = 106.6662267, alt = 7.0;     // target GPS

    GeographicLib::LocalCartesian proj(lat0, long0, alt0);  // khởi tạo hệ ENU

    double x, y, z;
    proj.Forward(line[0], line[1], line[2], x, y, z);   // GPS → ENU

    // std::cout << "x (East): " << x << ", y (North): " << y << ", z (Up): " << z << std::endl;
    std::stringstream oss;
    oss << x << "," << y << "," << z;
    return oss.str();
}
 
void ConvertGPStoENU::read_csv_file (const std::string& csv_to_read_path, const std::string& csv_to_write_path,
    const float& lat0, const float& long0, const float& alt0)
{
    std::ifstream file (csv_to_read_path);
    std::cout<<csv_to_read_path<<std::endl;
    if (!file.is_open()){
        std::cout<<"ERROR: Can not open csv file"<< std::endl;
        // RCLCPP_ERROR(this->get_logger(), "❌ Cannot open waypoints_log.csv");
        return;
    }
    if (file.is_open()){
        std::cout<<"INFO: CSV file is open, and ready to transfer."<<std::endl;}
    
    std::string line;
    while(std::getline(file, line))
    {
        std::stringstream single_stream_(line);
        std::string token;
        std::getline(single_stream_, token, ',');  // đọc và bỏ token đầu (index)
        std::vector<double> values;
        while(std::getline(single_stream_, token, ','))
        {
            values.push_back(std::stod(token));
            // std::cout << token << std::endl;
        }
        auto convert_values = convertGPS2ENU_function(lat0, long0, alt0, values);
        std::cout << convert_values << std::endl;
        // std::cout << values[1] << std::endl;
        // std::cout << values[2] << std::endl;
        write_csv_file(csv_to_write_path, convert_values);
        
        std::cout<<std::endl;
    }
    
    
    file.close();
}
// std::ofstream file ("/home/uyen/Drone/drone_ws/src/offboard_control/src/waypoint_log.csv", std::ios::trunc);
void ConvertGPStoENU::write_csv_file(const std::string& csv_path_to_write, std::string& line)
{
    //  std::cout << line << std::endl;
    // int order;
    // float X, Y, Z;
    // order = line[0];
    // X = line[1];
    // Y = line[2];
    // Z = line[3];
  
    std::ofstream file (csv_path_to_write, std::ios::app);
    if (!file.is_open()){
        std::cout<<"ERROR: Can not open convert_waypoint_log.csv file"<<std::endl;
        return;
    }
    if (file.is_open()){std::cout<<"INFO: CSV file is open, and ready to write."<<std::endl;}
    // file <<"Order, Latitude, Longitude, Altitude\n";
    file << line << "\n";
    file.close();
    std::cout << "✅ CSV written successfully.\n";
    return;
}
int main(int argc, char *argv[]) {
    ConvertGPStoENU convertor;
    double lat0 = 11.052945, long0 = 106.666147, alt0 = 0.0;
    std::string csv_path="/home/uyen/Drone/drone_ws/src/offboard_control/src/waypoint_log.csv";
    std::string csv_path_to_write="/home/uyen/Drone/drone_ws/src/offboard_control/src/transfer_waypoint_log.csv";
    convertor.read_csv_file(csv_path, csv_path_to_write, lat0, long0, alt0);
    return 0;
}