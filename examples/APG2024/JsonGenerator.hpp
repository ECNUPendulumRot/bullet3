

#ifndef JSON_GENERATOR_HPP
#define JSON_GENERATOR_HPP

#include "json.hpp"

#include "BulletDynamics/Dynamics/btRigidBody.h"

class JsonGenerator {

public:

	nlohmann::json metaJson;
	nlohmann::json framesJson;

	std::string jsonFileNamePrefix;

	static std::filesystem::path dir;

	JsonGenerator(const char* name) : jsonFileNamePrefix(name) {
		metaJson = nlohmann::json::array();
		framesJson = nlohmann::json::array();
	}

	void addObjects(btRigidBody** bodies, std::string* names, int bodyCount);

	void recordFrame(btRigidBody** bodies, int bodyCount);

	void saveToFile();

	static void createDirectory();

	int getFileCount() const;

	~JsonGenerator() {
		saveToFile();
	}
};


#endif