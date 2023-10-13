#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../Utils/b3BulletDefaultFileIO.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h>  //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "CableDemo.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCable/btCable.h"

#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include <iostream>

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///collisions between two btSoftBody's
class btSoftSoftCollisionAlgorithm;

///collisions between a btSoftBody and a btRigidBody
class btSoftRididCollisionAlgorithm;
class btSoftRigidDynamicsWorld;

class CableDemo : public CommonRigidBodyBase
{
public:
	btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;

	btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;

	btSoftBodyWorldInfo m_softBodyWorldInfo;

	bool m_autocam;
	bool m_cutting;
	bool m_raycast;
	btScalar m_animtime;
	btClock m_clock;
	int m_lastmousepos[2];
	btVector3 m_impact;
	btSoftBody::sRayCast m_results;
	btSoftBody::Node* m_node;
	btVector3 m_goal;
	bool m_drag;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	btBroadphaseInterface* m_broadphase;

	//btCollisionDispatcherMt* m_dispatcher;
	btCollisionDispatcher* m_dispatcher;

	btConstraintSolver* m_solver;

	btCollisionAlgorithmCreateFunc* m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

private:
	int m_currentDemoIndex;
	bool m_applyForceOnRigidbody;

	bool m_printFPS;
	clock_t current_ticks, delta_ticks;
	clock_t m_fps = 0;

	btVector3 m_cameraStartPosition;


public:
	void initPhysics();

	void exitPhysics();

	virtual void resetCamera()
	{
		//@todo depends on current_demo?
		float dist = 10;
		float pitch = 0;
		float yaw = 180;
		float targetPos[3] = {(float)m_cameraStartPosition.x(), (float)m_cameraStartPosition.y(), (float)m_cameraStartPosition.z()};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void SetCameraPosition(btVector3 cameraPosition)
	{
		m_cameraStartPosition = cameraPosition;
	}

	CableDemo(struct GUIHelperInterface* helper): CommonRigidBodyBase(helper),
		  m_drag(false)

	{
		m_applyForceOnRigidbody = false;
		m_printFPS = false;
		m_cameraStartPosition = btVector3(0, 3, 0);
	}
	virtual ~CableDemo()
	{
		btAssert(m_dynamicsWorld == 0);
	}

	//virtual void clientMoveAndDisplay();

	//virtual void displayCallback();

	void createStack(btCollisionShape* boxShape, float halfCubeSize, int size, float zPos);

	virtual void setDrawClusters(bool drawClusters);

	virtual const btSoftRigidDynamicsWorld* getSoftDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	//
	//void	clientResetScene();
	void renderme();
	bool keyboardCallback(int key, int state) override
	{
		if (key == 'a' && state)
		{
			m_applyForceOnRigidbody = !m_applyForceOnRigidbody;
		}

		if (key == 'd' && state)
		{
			PrintDistance_DemoCableForce();
		}

		if (key == 'f' && state)
		{
			m_printFPS = ! m_printFPS;
		}

		return false;
	}

	void mouseFunc(int button, int state, int x, int y);
	void mouseMotionFunc(int x, int y);

	GUIHelperInterface* getGUIHelper()
	{
		return m_guiHelper;
	}

	virtual void renderScene()
	{
		CommonRigidBodyBase::renderScene();
		btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

		for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
			//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
			}
		}
	}

	void stepSimulation(float deltaTime) override
	{
		if (m_dynamicsWorld)
		{
			if ( m_applyForceOnRigidbody)
			{
				AddConstantForce_DemoCableForce();
			}
			if (m_currentDemoIndex == 1 && m_applyForceOnRigidbody)
			{
				AddConstantForce_DemoCableForceUp();
			}

			current_ticks = clock();

			m_dynamicsWorld->stepSimulation(deltaTime);
		
			delta_ticks = clock() - current_ticks;
			
			if (m_printFPS)
			if (m_printFPS && delta_ticks > 0)
			{
				m_fps = CLOCKS_PER_SEC / delta_ticks;
				b3Printf("FPS: %d ", m_fps);
			}
		}
	}

	void AddConstantForce_DemoCableForce()
	{
		btCollisionObjectArray collisionArray = getSoftDynamicsWorld()->getCollisionObjectArray();
		for (int i = 0; i < 5 ; i++)
		{
			int index = 1 + i * 3;
			if (index < collisionArray.size())
			{
				float force = btPow(10, (i + 1));
				btRigidBody* rb = (btRigidBody*)collisionArray[index];
				rb->applyCentralForce(btVector3(0, 0, force));
			}
		}
	}

	void AddConstantForce_DemoCableForceUp()
	{
		btCollisionObjectArray collisionArray = getSoftDynamicsWorld()->getCollisionObjectArray();
		for (int i = 0; i < 5; i++)
		{
			int index = 0 + i * 3;
			if (index < collisionArray.size())
			{
				float force = btPow(10, (i + 1));
				btRigidBody* rb = (btRigidBody*)collisionArray[index];
				rb->applyCentralForce(btVector3(0, 0, force));
			}
		}
	}

	void PrintDistance_DemoCableForce()
	{
		btCollisionObjectArray collisionArray = getSoftDynamicsWorld()->getCollisionObjectArray();
		std::cout << "---" << std::endl;
		for (int i = 0; i < 5; i++)
		{
			int index = 2 + i * 3;
			if (index < collisionArray.size())
			{
				btCable* cable = (btCable*)collisionArray[index];
				PrintDistance(i, cable);
			}
		}
	}

	void PrintDistance(int indexCable, btCable * cable)
	{
		btSoftBody::Anchor a0 = cable->m_anchors[0];
		btVector3 worldPositionAnchor0 = a0.m_body->getCenterOfMassPosition() + a0.m_c1;
		btVector3 worldPositionNode0 = a0.m_node->m_x;

		btSoftBody::Anchor a1 = cable->m_anchors[1];
		btVector3 worldPositionAnchor1 = a1.m_body->getCenterOfMassPosition() + a1.m_c1;
		btVector3 worldPositionNode1 = a1.m_node->m_x;

		float distance0 = worldPositionAnchor0.distance(worldPositionNode0) * 100; // Convert in cm
		float distance1 = worldPositionAnchor1.distance(worldPositionNode1) * 100;  // Convert in cm

		b3Printf("Cable : % i - Distance Anchor0-Node %f cm | Distance Anchor1-Node %f cm | - Cable length %f | - Impluse: %f N", indexCable, distance0, distance1, cable->getLengthPosition(), cable->getImpulse(0).length());
	}

	void createCable(int resolution, int iteration, btScalar totalMass, btVector3 posAnchorKinematic, btVector3 posAnchorPhysic, btRigidBody* physic, btRigidBody* kinematic)
	{
		// Nodes' positions
		btVector3* positionNodes = new btVector3[resolution];
		btScalar* massNodes = new btScalar[resolution];
		for (int i = 0; i < resolution; ++i)
		{
			const btScalar t = i / (btScalar)(resolution - 1);
			positionNodes[i] = lerp(posAnchorPhysic, posAnchorKinematic, t);
			massNodes[i] = 1;
		}

		// Cable's creation
		btCable* cable = new btCable(&m_softBodyWorldInfo, getSoftDynamicsWorld(), resolution, positionNodes, massNodes);
		cable->setUseCollision(false);
		cable->appendAnchor(0, physic);
		cable->appendAnchor(cable->m_nodes.size() - 1, kinematic);
		// Cable's config
		cable->setTotalMass(totalMass);
		cable->m_cfg.piterations = iteration;
		cable->m_cfg.kAHR = 1;

		// Add cable to the world
		getSoftDynamicsWorld()->addSoftBody(cable);
	}
};

static void Init_Nodes(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 10;
	int iteration = 100;

	// Create 12 cube and 6 cables
	for (int i = 0; i < 5; i++)
	{
		if (i == 1)
		{
			resolution = 50;
		}
		else if (i == 2)
		{
			resolution = 100;
		}
		else if (i == 3)
		{
			resolution = 500;
		}
		else if (i == 4)
		{
			resolution = 1000;
		}

		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration, resolution, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_Weigths(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(1);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Create 12 cube and 6 cables
	for (int i = 0; i < 6; i++)
	{
		if (i > 0)
		{
			massPhysic = btPow(10, i);
		}

		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration,1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_Iterations(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(1000);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 1;

	// Create 12 cube and 6 cables
	for (int i = 0; i < 5; i++)
	{
		if (i > 0)
		{
			iteration = btPow(10, i);
		}

		// Positions
		btVector3 positionKinematic(-5 +( i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration,1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_Lengths(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Positions
	btVector3 positionKinematic(-5, 7, 0);

	// Create 12 cube and 6 cables
	for (int i = 0; i < 5; i++)
	{
		if (i == 1)
		{
			positionKinematic = btVector3(-2.5, 10, 0);
		}
		else if (i == 2)
		{
			positionKinematic = btVector3(0, 30, 0);
		}
		else if (i == 3)
		{
			positionKinematic = btVector3(2.5, 105, 0);
		}
		else if (i == 4)
		{
			positionKinematic = btVector3(5, 505, 0);
		}

		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration, 1,  anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_CableForceDown(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	
	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Create 10 cube and 5 cables
	for (int i = 0 ; i < 5 ; i++)
	{
		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);
		
		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);
		
		pdemo->createCable(resolution, iteration,1,  anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
		
	}
}

static void Init_CableForceUp(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	// Cable 0: 10 nodes ; 5 m ; 2 bodies (0 & 10 kg) ; static
	//
	// Masses
	btScalar massKinematic(10);
	btScalar massPhysic(10);

	// Rotation
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	pdemo->m_softBodyWorldInfo.m_gravity = btVector3(0, 0, 0);
	pdemo->getSoftDynamicsWorld()->setGravity(btVector3(0, 0, 0));

	// Create 10 cube and 5 cables
	for (int i = 0; i < 5; i++)
	{
		// Positions
		btVector3 positionKinematic(-5 + (i * 2.5f), 10, 0);
		btVector3 positionPhysic(-5 + (i * 2.5f), 4, 0);
		transformKinematic.setOrigin(positionKinematic);
		transformPhysic.setOrigin(positionPhysic);

		// Create the rigidbodys
		btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, boxShape);
		btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, boxShape);

		kinematic->clearGravity();
		physic->clearGravity();

		// Anchor's positions
		btVector3 anchorPositionKinematic = positionKinematic - btVector3(0, -0.5, 0);
		btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

		pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	}
}

static void Init_TestArse(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* supportShape = new btBoxShape(btVector3(16, 2, 2));
	btCollisionShape* arseShape = new btBoxShape(btVector3(13, 5, 5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(750);

	// Position / Rotation
	btVector3 positionKinematic(0, 20, 0);
	btVector3 positionPhysic(0, 10, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, supportShape);
	btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, arseShape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(-3.8, -2.0, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(-4, 5, 0);

	pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);

	// Anchor's positions
	anchorPositionKinematic = positionKinematic + btVector3(3.8, -2.0, 0);
	anchorPositionPhysic = positionPhysic + btVector3(4, 5, 0);

	pdemo->createCable(resolution, iteration, 1, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);

	pdemo->SetCameraPosition(btVector3(0, 17, 15));
}

static void Init_TestSupportT18(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(777);

	// Position / Rotation
	btVector3 positionKinematic(0, 10, 0);
	btVector3 positionPhysic(0, 4, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	// Resolution's cable
	int resolution = 20;
	int iteration = 100;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, shape);
	btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, shape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(0, -0.5, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

	pdemo->createCable(resolution, iteration, 1.61, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
}

static void Init_TestSupportA18(CableDemo* pdemo)
{
	// Shape
	btCollisionShape* shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));

	// Masses
	btScalar massKinematic(0);
	btScalar massPhysic(404);

	// Position / Rotation
	btVector3 positionKinematic(0, 16, 0);
	btVector3 positionPhysic(0, 0, 0);
	btQuaternion rotation(0, 0, 0, 1);

	// Transform
	btTransform transformKinematic;
	transformKinematic.setIdentity();
	transformKinematic.setRotation(rotation);
	transformKinematic.setOrigin(positionKinematic);

	btTransform transformPhysic;
	transformPhysic.setIdentity();
	transformPhysic.setRotation(rotation);
	transformPhysic.setOrigin(positionPhysic);

	// Resolution's cable
	int resolution = 20;
	int iteration = 50;

	// Create the rigidbodys
	btRigidBody* kinematic = pdemo->createRigidBody(massKinematic, transformKinematic, shape);
	btRigidBody* physic = pdemo->createRigidBody(massPhysic, transformPhysic, shape);

	// Anchor's positions
	btVector3 anchorPositionKinematic = positionKinematic + btVector3(0, -0.5, 0);
	btVector3 anchorPositionPhysic = positionPhysic + btVector3(0, 0.5, 0);

	btScalar cableTotalMass(15.0 * 0.243);

	pdemo->createCable(resolution, iteration, cableTotalMass, anchorPositionKinematic, anchorPositionPhysic, physic, kinematic);
	pdemo->SetCameraPosition(btVector3(0, 0.5, -9));
}

void (*demofncs[])(CableDemo*) =
{
		Init_CableForceDown,
		Init_CableForceUp,
		Init_Nodes,
		Init_Weigths,
		Init_Iterations,
		Init_Lengths,
		Init_TestArse,
		Init_TestSupportT18, 
		Init_TestSupportA18};

////////////////////////////////////
///for mouse picking
void pickingPreTickCallbackCable(btDynamicsWorld* world, btScalar timeStep)
{
	CableDemo* cableDemo = (CableDemo*)world->getWorldUserInfo();

	if (cableDemo->m_drag)
	{
		const int x = cableDemo->m_lastmousepos[0];
		const int y = cableDemo->m_lastmousepos[1];
		float rf[3];
		cableDemo->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraPosition(rf);
		float target[3];
		cableDemo->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraTargetPosition(target);
		btVector3 cameraTargetPosition(target[0], target[1], target[2]);

		const btVector3 cameraPosition(rf[0], rf[1], rf[2]);
		const btVector3 rayFrom = cameraPosition;

		const btVector3 rayTo = cableDemo->getRayTo(x, y);
		const btVector3 rayDir = (rayTo - rayFrom).normalized();
		const btVector3 N = (cameraTargetPosition - cameraPosition).normalized();
		const btScalar O = btDot(cableDemo->m_impact, N);
		const btScalar den = btDot(N, rayDir);
		if ((den * den) > 0)
		{
			const btScalar num = O - btDot(N, rayFrom);
			const btScalar hit = num / den;
			if ((hit > 0) && (hit < 1500))
			{
				cableDemo->m_goal = rayFrom + rayDir * hit;
			}
		}
		btVector3 delta = cableDemo->m_goal - cableDemo->m_node->m_x;
		static const btScalar maxdrag = 10;
		if (delta.length2() > (maxdrag * maxdrag))
		{
			delta = delta.normalized() * maxdrag;
		}
		cableDemo->m_node->m_v += delta / timeStep;
	}
}

int currentCableDemo = 0;

///
/// btTaskSchedulerManager -- manage a number of task schedulers so we can switch between them
///
class btTaskSchedulerManager
{
	btAlignedObjectArray<btITaskScheduler*> m_taskSchedulers;
	btAlignedObjectArray<btITaskScheduler*> m_allocatedTaskSchedulers;

public:
	btTaskSchedulerManager() {}
	void init()
	{
		addTaskScheduler(btGetSequentialTaskScheduler());
#if BT_THREADSAFE
		if (btITaskScheduler* ts = btCreateDefaultTaskScheduler())
		{
			m_allocatedTaskSchedulers.push_back(ts);
			addTaskScheduler(ts);
		}
		addTaskScheduler(btGetOpenMPTaskScheduler());
		addTaskScheduler(btGetTBBTaskScheduler());
		addTaskScheduler(btGetPPLTaskScheduler());
		if (getNumTaskSchedulers() > 1)
		{
			// prefer a non-sequential scheduler if available
			btSetTaskScheduler(m_taskSchedulers[1]);
		}
		else
		{
			btSetTaskScheduler(m_taskSchedulers[0]);
		}
#endif  // #if BT_THREADSAFE
	}
	void shutdown()
	{
		for (int i = 0; i < m_allocatedTaskSchedulers.size(); ++i)
		{
			delete m_allocatedTaskSchedulers[i];
		}
		m_allocatedTaskSchedulers.clear();
	}

	void addTaskScheduler(btITaskScheduler* ts)
	{
		if (ts)
		{
#if BT_THREADSAFE
			// if initial number of threads is 0 or 1,
			if (ts->getNumThreads() <= 1)
			{
				// for OpenMP, TBB, PPL set num threads to number of logical cores
				ts->setNumThreads(ts->getMaxNumThreads());
			}
#endif  // #if BT_THREADSAFE
			m_taskSchedulers.push_back(ts);
		}
	}
	int getNumTaskSchedulers() const { return m_taskSchedulers.size(); }
	btITaskScheduler* getTaskScheduler(int i) { return m_taskSchedulers[i]; }
};

static btTaskSchedulerManager gTaskSchedulerMgr;

void CableDemo::setDrawClusters(bool drawClusters)
{
	if (drawClusters)
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags() | fDrawFlags::Clusters);
	}
	else
	{
		getSoftDynamicsWorld()->setDrawFlags(getSoftDynamicsWorld()->getDrawFlags() & (~fDrawFlags::Clusters));
	}
}

void CableDemo::initPhysics()
{
	m_currentDemoIndex = currentCableDemo;

	///create concave ground mesh
	m_guiHelper->setUpAxis(1);

	m_dispatcher = 0;

	if (gTaskSchedulerMgr.getNumTaskSchedulers() == 0)
	{
		gTaskSchedulerMgr.init();
	}

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	//m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_dispatcher = new btCollisionDispatcherMt(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm

	///Register softbody versus rigidbody collision algorithm

	////////////////////////////

	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	//m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	m_broadphase = new btDbvtBroadphase();

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	//btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	btSequentialImpulseConstraintSolverMt* solver = new btSequentialImpulseConstraintSolverMt();

	m_solver = solver;

	btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

	static bool once = true;
	if (once)
	{
		once = false;
		initCL(0, 0);
	}

	if (g_openCLSIMDSolver)
		delete g_openCLSIMDSolver;
	if (g_softBodyOutput)
		delete g_softBodyOutput;

	if (1)
	{
		g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware(g_cqCommandQue, g_cxMainContext);
		//	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
		g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
	}

	softBodySolver = g_openCLSIMDSolver;
	g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif  //USE_AMD_OPENCL

	//btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
	m_dynamicsWorld = world;
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallbackCable, this, true);

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
	m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);
	m_guiHelper->createPhysicsDebugDrawer(world);
	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	//	clientResetScene();

	//create ground object

	int lastDemo = (sizeof(demofncs) / sizeof(demofncs[0])) - 1;

	if (currentCableDemo < 0)
		currentCableDemo = lastDemo;
	if (currentCableDemo > lastDemo)
		currentCableDemo = 0;

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	m_softBodyWorldInfo.air_density = (btScalar)0;
	m_softBodyWorldInfo.water_density = 0;
	m_softBodyWorldInfo.water_offset = 0;
	m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
	m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);

	m_autocam = false;
	m_raycast = false;
	m_cutting = false;
	m_results.fraction = 1.f;

	demofncs[currentCableDemo](this);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CableDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

class CommonExampleInterface* CableDemoCreateFunc(struct CommonExampleOptions& options)
{
	currentCableDemo = options.m_option;
	return new CableDemo(options.m_guiHelper);
}
