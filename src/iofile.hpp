#ifndef __IO_FILE_HPP__
#define __IO_FILE_HPP__

// Windows
#include <direct.h>
#include <io.h>

// Linux
//#include <sys/io.h>	
//#include <unistd.h>
//#include <sys/stat.h>
//#include <sys/types.h>
//#include <dirent.h>

#include <filesystem>
#include <fstream>
#include "utilities.hpp"
#include "lsd.hpp" 

namespace fs = std::filesystem;


void listDirRecursively(
	std::vector<std::string>& filenames,
	const std::string&        dir,
	const std::string&        suffix
);


void listDir(
	std::vector<std::string>& filenames,
	const std::string&        dir,
	const std::string&        suffix
);


void readFileNames(
	const std::string&        filepath,
	std::vector<std::string>& filenames
);


void write2txt(
	LineSegList&       segments, 
	const std::string& dir, 
	const std::string& filename,
	const std::string& suffix = ".csv"
);


#endif // !__IO_FILE_HPP__

