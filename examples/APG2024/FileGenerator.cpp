

#include "FileGenerator.hpp"
#include <string>


std::filesystem::path FileGenerator::dir = "BodyInfo";


void FileGenerator::createDirectory() {
	if (std::filesystem::exists(dir)) {
		return;
	}
	std::filesystem::create_directory(dir);
}

int FileGenerator::getFileCount() {
	int count = 0;
	std::filesystem::directory_entry entry(dir);
	std::filesystem::directory_iterator files(entry);

	for(auto& file : files) {
		std::string fileName = file.path().filename().string();

		if (fileName.find(csvFilePrefix.string()) != std::string::npos) {
			count++;
		}
	}

	return count;
}


void FileGenerator::openFile(std::string* names, int count) {

	if (fileIsOpen) {
		return;
	}

	int fileCount = getFileCount();
	std::filesystem::path filePath = dir / (csvFilePrefix.string() + "_" + std::to_string(fileCount) + ".csv");

	csvFile.open(filePath);

	if (!csvFile.is_open()) {
		std::cerr << "Failed to open file: " << filePath.string() << std::endl;
		return;
	}

	fileIsOpen = true;

	for (int i = 0; i < count; i++) {
		csvFile << names[i] + ","
				<< "p_x,"
				<< "p_y,"
				<< "p_z,"
				<< "q_x,"
				<< "q_y,"
				<< "q_z,"
				<< "q_w,"
				<< "v_x,"
				<< "v_y,"
				<< "v_z,"
				<< "w_x,"
				<< "w_y,"
				<< "w_z, ,";
	}
	csvFile << std::endl;
}


void FileGenerator::saveBodyInfo(btRigidBody** bodies, std::string* names, int count)
{
	openFile(names, count);

	for (int i = 0; i < count; i++) {
		btRigidBody* body = bodies[i];

		btVector3 p = body->getCenterOfMassPosition();
		btQuaternion q = body->getOrientation();
		btVector3 v = body->getLinearVelocity();
		btVector3 w = body->getAngularVelocity();

		csvFile << "," << p.x() << "," << p.y() << "," << p.z() << ",";
		csvFile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ",";
		csvFile << v.x() << "," << v.y() << "," << v.z() << ",";
		csvFile << w.x() << "," << w.y() << "," << w.z() << ", ,";
	}
	csvFile << std::endl;
}
