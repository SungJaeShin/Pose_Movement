#include "include.h"
#include "utility.h"

// For file format following as: x, y, z, qw, qx, qy, qz (CSV format !!)
std::vector<double> parseCSVfile(std::istream &file){
	std::vector<double> parse;
	std::string line;

	double val;
	while(std::getline(file, line, ',')){
		std::stringstream s_line(line);

		while(s_line >> val){
			parse.push_back(val);

			if(s_line.peek() == ',') s_line.ignore();
		}
	}
	return parse;
}

// For file format following as: time, x, y, z, qx, qy, qz, qw (TXT format !!)
std::vector<double> parseTXTfile(std::istream &file){
    std::vector<double> parse;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream s_line(line);
        double val;

        // Skip first line which informs descriptions
        std::string cell;
        s_line >> cell;
        std::stringstream s_cell(cell);
        if (!(s_cell >> val)) {
            std::cout << "Skipping line: " << line << std::endl;
            continue;
        }
        parse.push_back(val);

        while (s_line >> val) {
            parse.push_back(val);
        }
    }
    return parse;
}

// Read PLY file using PCL library
pcl::PointCloud<pcl::PointXYZ>::Ptr readPLYfile(std::string &file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file, *cloud) == -1) 
    {
        PCL_ERROR("Couldn't read the ply file\n");
        return cloud;
    }
    return cloud;
}