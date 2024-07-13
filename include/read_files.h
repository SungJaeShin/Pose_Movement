#include "include.h"

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
std::vector<double> parseTXTfile(std::istream &file) {
    std::vector<double> parse;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream s_line(line);
        double val;

        while (s_line >> val) {
            parse.push_back(val);
        }
    }
    return parse;
}
