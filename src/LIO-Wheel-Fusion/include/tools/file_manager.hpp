#ifndef SRC_TOOLS_FILE_MANAGER_HPP_
#define SRC_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

class FileManager {
public:
    static bool CreateFile(std::ofstream &ofs, std::string file_path);

    static bool InitDirectory(std::string directory_path, std::string use_for);

    static bool CreateDirectory(std::string directory_path, std::string use_for);

    static bool CreateDirectory(std::string directory_path);
};

#endif
