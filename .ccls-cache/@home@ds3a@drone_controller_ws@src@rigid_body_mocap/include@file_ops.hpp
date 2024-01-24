#ifndef FILE_OPS_H_
#define FILE_OPS_H_

#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace file_ops {
    uint8_t write_tf_tree(std::vector<std::vector<Eigen::Vector3d>> vec_to_write,
                          std::string filename) {

        std::ofstream outfile(filename);
        for (const std::vector<Eigen::Vector3d>& innerVector : vec_to_write) {
            for (const Eigen::Vector3d &vector3d : innerVector) {
                outfile << vector3d.x() << "," << vector3d.y() << "," << vector3d.z()
                        << "\n";
            }
            outfile << "\n"; // Separate inner vectors
        }
        outfile.close();
        return 1;
    }

    std::vector<std::vector<Eigen::Vector3d>> read_tf_tree(std::string filename) {
        std::ifstream infile(filename);  // Replace with your file name

        std::vector<std::vector<Eigen::Vector3d>> myArray;
        std::vector<Eigen::Vector3d> currentVector;
        std::string line;

        while (std::getline(infile, line)) {
            if (line.empty()) {  // Check for empty line to separate inner vectors
                if (!currentVector.empty()) {  // Add non-empty current vector
                    myArray.push_back(currentVector);  // Append current vector to myArray
                    currentVector.clear();  // Reset for next inner vector
                }
                continue;
            }

            std::string valueStr;
            std::stringstream ss(line);
            std::vector<double> values;
            while (std::getline(ss, valueStr, ',')) {  // Assuming comma-separated
                values.push_back(std::stod(valueStr));
            }

            if (values.size() == 3) {  // Ensure correct number of values for a 3D vector
                currentVector.push_back(Eigen::Vector3d(values[0], values[1], values[2]));
            } else {
                std::cerr << "Invalid line format: " << line << std::endl;
            }
        }

        // Add the last vector if it's not empty (even without an empty line at the end)
        if (!currentVector.empty()) {
            myArray.push_back(currentVector);
        }

        return myArray;
    }


    uint8_t write_graph_to_marker_map(std::map<int, int> map, std::string filename) {
        std::ofstream outputFile(filename);

        for(const auto& pair : map) {
            outputFile << pair.first << ' ' << pair.second << '\n';
        }
        outputFile.close();
    }

    std::map<int, int> read_graph_to_marker_map(std::string filename) {
        std::map<int, int> loadedData;
        std::ifstream inputFile(filename);
        int key;
        int value;

        while(inputFile >> key >> value) {
            loadedData[key] = value;
        }
        inputFile.close();

        return loadedData;
    }
} // namespace file_ops

#endif // FILE_OPS_H_
