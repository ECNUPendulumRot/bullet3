
#ifndef FILEGENERATOR_HPP
#define FILEGENERATOR_HPP

#include <iostream>
#include <fstream>
#include <filesystem>

#include "BulletDynamics/Dynamics/btRigidBody.h"

class FileGenerator {

	static std::filesystem::path dir;
	std::filesystem::path csvFilePrefix;

	std::ofstream csvFile;

	bool fileIsOpen = false;
public:

	FileGenerator(const char* prefix) : csvFilePrefix(prefix) {
		createDirectory();
	}

	void createDirectory();

	int getFileCount();

	void openFile(std::string* names, int count);

	void saveBodyInfo(btRigidBody** bodies, std::string* names, int count);

	void closeFile() {
		if (fileIsOpen) {
			csvFile.close();
		}
	}

	~FileGenerator() {
		closeFile();
	}
};

#endif
