#include "iofile.hpp"


void listDirRecursively(
	std::vector<std::string>& filenames,
	const std::string& dir,
	const std::string& suffix
)
{
	try {
		if (fs::exists(dir) && fs::is_directory(dir))
		{
			for (const auto& entry : fs::recursive_directory_iterator(dir))
			{
				if (fs::is_regular_file(entry.status()))
				{
					std::string name = entry.path().filename().string();
					size_t ind = name.find_first_of('.');
					if (name.substr(ind) == suffix)
					{
						filenames.push_back(dir + '/' + name.substr(0, ind) + '/' + name);
					}
				}
			}
		}
		else {
			std::cerr << "The directory does not exist or is not a directory." << std::endl;
			return;
		}
	}
	catch (const fs::filesystem_error& e) {
		std::cerr << e.what() << std::endl;
		return;
	}

	return;
}


void listDir(
	std::vector<std::string>& filenames,
	const std::string& dir,
	const std::string& suffix
)
{
	try {
		if (fs::exists(dir) && fs::is_directory(dir))
		{
			for (const auto& entry : fs::directory_iterator(dir))
			{
				if (fs::is_regular_file(entry.status()))
				{
					std::string name = entry.path().filename().string();
					size_t ind = name.find_first_of('.');
					if (name.substr(ind) == suffix)
					{
						filenames.push_back(dir + '/' + name);
					}
				}
			}
		}
		else {
			std::cerr << "The directory does not exist or is not a directory." << std::endl;
			return;
		}
	}
	catch (const fs::filesystem_error& e) {
		std::cerr << e.what() << std::endl;
		return;
	}

	return;
}


void readFileNames(
	const std::string&        filepath,
	std::vector<std::string>& filenames
)
{
	std::fstream fs(filepath, std::ios::in);
	if (!fs.is_open())
	{
		std::cerr << "No file was found." << std::endl;
		return;
	}

	filenames.clear();
	std::string filename;
	while (std::getline(fs, filename))
	{
		filenames.emplace_back(filename);
	}

	return;
}


void write2txt(
	LineSegList&       segments, 
	const std::string& dir,
	const std::string& filename,
	const std::string& suffix
)
{
	if (_access(dir.c_str(), 0) == -1)
	{
		int flag = _mkdir(dir.c_str());
		if (flag == -1)
		{
			std::cerr << "Creating directory failed." << std::endl;
			return;
		}
	}

	std::string imgname = filename.substr(filename.find_last_of('/') + 1);
	imgname = imgname.substr(0, imgname.find_last_of('.'));

	std::fstream fs(dir + "/" + imgname + suffix, std::ios::out | std::ios::trunc);

	for (const auto& seg : segments)
	{
		fs << seg.begPx.x << "," << seg.begPx.y << ","
			<< seg.endPx.x << "," << seg.endPx.y << '\n';
	}
	fs.close();
}