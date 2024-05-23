
#include "Cradle.hpp"
#include "FileGenerator.hpp"
#include "JsonGenerator.hpp"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct CradleExample : public CommonRigidBodyBase {

	btRigidBody* bodies[1024];
	std::string names[1024];

    // modify y-axis velocity
    btVector3 velocity{0, 5, 0};

	int sphereCount = 4;

	FileGenerator* fileGenerator;

	JsonGenerator* jsonGenerator;

	CradleExample(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper) {
		fileGenerator = new FileGenerator("Cradle");
		jsonGenerator = new JsonGenerator("Scene");
	}

	virtual ~CradleExample() {
		delete fileGenerator;
		delete jsonGenerator;
	}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 20;
		float pitch = -60;
		float yaw = 0;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};


void CradleOutputBodyInfo(btDynamicsWorld* world, btScalar deltaTime) {
	CradleExample* example = (CradleExample*)world->getWorldUserInfo();

	btRigidBody** bodies = example->bodies;
	// 1 is dynamic body
	int bodyCount = example->sphereCount + 1;
	std::string* names = example->names;

	FileGenerator* fileGenerator = example->fileGenerator;
	fileGenerator->saveBodyInfo(bodies, names, bodyCount);

	JsonGenerator* jsonGenerator = example->jsonGenerator;
	jsonGenerator->recordFrame(bodies, bodyCount);
}


void CradleExample::initPhysics()
{
	m_guiHelper->setUpAxis(2);

	createEmptyDynamicsWorld();

	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer()) {
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	}

	btBoxShape* groundShape = new btBoxShape(btVector3(100, 100, 100));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, -100));

	btRigidBody* groundBody = createRigidBody(0, groundTransform, groundShape);
	groundBody->setFriction(0);
	groundBody->setRestitution(1);
	groundBody->setRollingFriction(0);
	groundBody->setSpinningFriction(0);

	btSphereShape* sphereShape = new btSphereShape(1);
	m_collisionShapes.push_back(sphereShape);

	btTransform sphereTransform;
	sphereTransform.setIdentity();
	sphereTransform.setOrigin(btVector3(0, -3, 1));

	btScalar mass = 1.f;
	btVector3 localInertia(0, 0, 0);

	sphereShape->calculateLocalInertia(mass, localInertia);

	bodies[0] = createRigidBody(mass, sphereTransform, sphereShape);
	names[0] = "init_velocity_sphere";

    bodies[0]->setLinearVelocity(velocity);

	for (int i = 1; i <= sphereCount; i++) {
		sphereTransform.setOrigin(btVector3(0, 1 + 2 * i, 1));
		bodies[i] = createRigidBody(mass, sphereTransform, sphereShape);

		names[i] = "sphere" + std::to_string(i);
	}

	for (int i = 0; i <= sphereCount; i++) {
		bodies[i]->setFriction(0);
		bodies[i]->setRollingFriction(0);
		bodies[i]->setSpinningFriction(0);
		bodies[i]->setDamping(0, 0);
		bodies[i]->setRestitution(1);
	}

	m_dynamicsWorld->setInternalTickCallback(CradleOutputBodyInfo, this, true);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	jsonGenerator->addObjects(bodies, names, sphereCount + 1); // 1 is dynamic body
}


void CradleExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}


CommonExampleInterface* CradleCreateFunc(CommonExampleOptions& options)
{
	return new CradleExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(CradleCreateFunc);