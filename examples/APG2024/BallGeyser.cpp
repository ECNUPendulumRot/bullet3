
#include "BallGeyser.hpp"
#include "FileGenerator.hpp"
#include "JsonGenerator.hpp"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

struct BallGeyserExample : public CommonRigidBodyBase {

	btRigidBody* bodies[1024];
	std::string names[1024];

    // modify y-axis velocity
    btVector3 velocity{0, 20, 0};

	int bodyCount = 8;

    // odd number
    int layer = 7;

	FileGenerator* fileGenerator;

	JsonGenerator* jsonGenerator;

	BallGeyserExample(struct GUIHelperInterface* helper) : CommonRigidBodyBase(helper) {
		fileGenerator = new FileGenerator("BallGeyser");
		jsonGenerator = new JsonGenerator("Scene");
	}

	virtual ~BallGeyserExample() {
		delete fileGenerator;
		delete jsonGenerator;
	}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 30;
		float pitch = -88;
		float yaw = 0;
		float targetPos[3] = {-5, 20, 1};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};


void BallGeyserOutputBodyInfo(btDynamicsWorld* world, btScalar deltaTime) {
	BallGeyserExample* example = (BallGeyserExample*)world->getWorldUserInfo();

	btRigidBody** bodies = example->bodies;
	int bodyCount = example->bodyCount;
	std::string* names = example->names;

	FileGenerator* fileGenerator = example->fileGenerator;
	fileGenerator->saveBodyInfo(bodies, names, bodyCount);

	JsonGenerator* jsonGenerator = example->jsonGenerator;
	jsonGenerator->recordFrame(bodies, bodyCount);
}


void BallGeyserExample::initPhysics()
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

    btVector3 centerPos(-5, 20, 1);
    btVector3 leftBoxPos(-20, 10, 1);
    btVector3 rightBoxPos(10, 10, 1);

	btSphereShape* sphereShape = new btSphereShape(1);
	m_collisionShapes.push_back(sphereShape);

	btTransform sphereTransform;
	sphereTransform.setIdentity();
	sphereTransform.setOrigin(btVector3(centerPos.x(), centerPos.y() - 14, 1));

	btScalar mass = 1.f;
	btVector3 localInertia(0, 0, 0);

	sphereShape->calculateLocalInertia(mass, localInertia);

	bodies[0] = createRigidBody(mass, sphereTransform, sphereShape);
	names[0] = "init_velocity_sphere";

    bodies[0]->setLinearVelocity(velocity);

    int index = 1;
    btScalar xOffset = 2.0;
    btScalar yOffset = sqrtf(3.0f);

    int SPHERE_COUNT[5] = {4, 3, 4, 3, 2};
    for (int i = 0; i < 5; i++) {
        btVector3 leftPos(centerPos.x() - SPHERE_COUNT[i] + 1.0, centerPos.y() - i * yOffset, 1);

        for(int j = 0; j < SPHERE_COUNT[i]; j++) {
            btVector3 pos(leftPos.x() + xOffset * j, leftPos.y(), 1);
            sphereTransform.setOrigin(pos);

            bodies[index] = createRigidBody(mass, sphereTransform, sphereShape);
            names[index] = "sphere_" + std::to_string(index);
            index++;
        }
    }

    for (int i = 1; i <= layer; i++) {
        btVector3 leftPos(centerPos.x() - layer + i, centerPos.y() + i * yOffset, 1);
        for(int j = 0; j<= layer - i; j++) {
            btVector3 pos(leftPos.x() + j * xOffset, leftPos.y(), 1);
            sphereTransform.setOrigin(pos);

            bodies[index] = createRigidBody(mass, sphereTransform, sphereShape);
            names[index] = "sphere_" + std::to_string(index);
            index++;
        }
    }

    // create two boxes
    btScalar hx = centerPos.x() - leftBoxPos.x() - 4;
    btScalar hy = centerPos.y() - leftBoxPos.y() + sqrtf(3.0f) - 1.f;
    btBoxShape* leftBoxShape = new btBoxShape(btVector3(hx, hy, 1));

    btTransform boxTransform;
    boxTransform.setIdentity();
    boxTransform.setOrigin(leftBoxPos);

    bodies[index] = createRigidBody(0, boxTransform, leftBoxShape);
    names[index] = "left_box";
    index++;

    hx = rightBoxPos.x() - centerPos.x() - 4;
    hy = centerPos.y() - rightBoxPos.y() + sqrtf(3.0f) - 1.f;
    btBoxShape* rightBoxShape = new btBoxShape(btVector3(hx, hy, 1));

    boxTransform.setOrigin(rightBoxPos);

    bodies[index] = createRigidBody(0, boxTransform, rightBoxShape);
    names[index] = "right_box";
    index++;

    bodyCount = index;

	for (int i = 0; i < bodyCount; i++) {
		bodies[i]->setFriction(0);
		bodies[i]->setRollingFriction(0);
		bodies[i]->setSpinningFriction(0);
		bodies[i]->setDamping(0, 0);
		bodies[i]->setRestitution(1);
	}

	m_dynamicsWorld->setInternalTickCallback(BallGeyserOutputBodyInfo, this, true);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	jsonGenerator->addObjects(bodies, names, bodyCount);
}


void BallGeyserExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}


CommonExampleInterface* BallGeyserCreateFunc(CommonExampleOptions& options)
{
	return new BallGeyserExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(CradleCreateFunc);