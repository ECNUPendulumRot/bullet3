
#include "JsonGenerator.hpp"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>

#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

std::filesystem::path JsonGenerator::dir = "JsonInfo";

void JsonGenerator::addObjects(btRigidBody** bodies, std::string* names, int bodyCount)
{
	using json = nlohmann::json;

	for (int i = 0; i < bodyCount; i++) {
		json objJson;

		objJson["name"] = names[i] + "_bullet3";
		btRigidBody* body = bodies[i];

		btCollisionShape* shape = body->getCollisionShape();
		switch (shape->getShapeType())
		{
			case BroadphaseNativeTypes::SPHERE_SHAPE_PROXYTYPE:
				objJson["type"] = "sphere";
				objJson["radius"] = ((btSphereShape*)shape)->getRadius();
				break;
            case BroadphaseNativeTypes::BOX_SHAPE_PROXYTYPE: {
                objJson["type"] = "box";
                btVector3 xyz = ((btBoxShape*)shape)->getHalfExtentsWithMargin() * 2;
                objJson["hx"] = xyz.x();
                objJson["hy"] = xyz.y();
                objJson["hz"] = xyz.z();
                break;
            }
			default:
				break;
		}

		metaJson.push_back(objJson);
	}
}

void JsonGenerator::recordFrame(btRigidBody** bodies, int bodyCount)
{
	using json = nlohmann::json;

	json frameJson;

	for (int i = 0; i < bodyCount; i++) {
		btRigidBody* body = bodies[i];

		btVector3 p = body->getCenterOfMassPosition();
		btQuaternion q = body->getOrientation();

		json statusJson;
		statusJson["position"] = {p.x(), p.y(), p.z()};
		statusJson["quaternion"] = {q.w(), q.x(), q.y(), q.z()};
		frameJson[metaJson[i]["name"]] = statusJson;
	}

	framesJson.push_back(frameJson);
}


void JsonGenerator::createDirectory()
{
	if (std::filesystem::exists(dir)) {
		return;
	}
	std::filesystem::create_directory(dir);
}

int JsonGenerator::getFileCount() const
{
	int count = 0;
	std::filesystem::directory_entry entry(dir);
	std::filesystem::directory_iterator files(entry);

	for(auto& file : files) {
		std::string fileName = file.path().filename().string();

		if (fileName.find(jsonFileNamePrefix) != std::string::npos) {
			count++;
		}
	}

	return count;
}


void JsonGenerator::saveToFile()
{
	createDirectory();

	// int fileCount = getFileCount();

	// std::filesystem::path fileName = dir / (jsonFileNamePrefix + "_" + std::to_string(fileCount) + ".json");
    std::filesystem::path fileName = dir / (jsonFileNamePrefix + ".json");

	std::ofstream file(fileName);

	nlohmann::json json;
	json["meta"] = metaJson;
	json["frames"] = framesJson;

	if (file.is_open()) {
		file << json.dump(4);
		file.close();
	} else {
		std::cerr << "Failed to open file " << fileName << std::endl;
	}
}