#include "iostream"
#include "GeographicLib/LocalCartesian.hpp"
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>



// using namespace GeographicLib;

class ConvertGPStoENU 
{
    public:
    ConvertGPStoENU();
    void read_csv_file (const std::string& csv_to_read_path, const std::string& csv_to_write_path, const float& lat0, 
                        const float& long0, const float& alt0);
    std::string convertGPS2ENU_function(double lat0, double long0, double alt0, std::vector<double> line);
    void write_csv_file(const std::string& csv_path_to_write, std::string& line);

    private:
    // float X, Y, Z;
    
};

