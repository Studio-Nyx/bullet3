#ifndef _BT_CABLE_H
#define _BT_CABLE_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMinMax.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <list>
#include <vector>
#include <omp.h>
#include <iostream>

#include "cubic_spline.hpp"


using namespace std;

class CableManifolds
{
public:
	btPersistentManifold* manifold;
	int lifeTime;
	CableManifolds(btPersistentManifold* mani, int time)
	{
		manifold = mani;
		lifeTime = time;
	}
};
	
///The btCable is a class that inherits from btSoftBody.
///Its purpose is to be able to create a cable/rope with our own method parameters that Bullet does not implement.
class btCable : public btSoftBody
{
	struct BroadPhasePair
	{
		btCollisionObject* body;
		bool haveManifoldsRegister;
		btPersistentManifold* manifold;
		btVector3 totalImpulse;
		btVector3 applicationPoint;
		int count;
	};

	struct NodePairNarrowPhase
	{
		Node* node;
		btVector3 m_Xout;
		btCollisionShape* collisionShape;
		BroadPhasePair* pair;
		btTransform worldToLocal;

		btVector3 lastPosition;
		btVector3 impulse;
		btVector3 normal;
		btScalar distance;
		bool hit = false;
		bool hitInIteration = false;
	};
	
	enum class CollisionMode
	{
		Linear = 0,
		Exponential
	};

	//
	~btCable()
	{
		int size = manifolds.size();
		for (int i = 0; i < size; i++)
		{
			m_world->getDispatcher()->releaseManifold(manifolds.at(i).manifold);
		}
		manifolds.clear();

		// Release allocated data
		if (m_cableData != nullptr)
		{
			delete m_cableData;
		}

		if (m_nodePos != nullptr)
		{
			delete[] m_nodePos;
		}

		if (m_nodeData != nullptr)
		{
			delete[] m_nodeData;
		}

		if (m_section != nullptr)
		{
			delete[] m_section;
		}
		delete spline;
	}

	btAlignedObjectArray<CableManifolds> manifolds;
	
private:
	
	int m_growingState=0;
	// Number of solverIteration for 1 deltaTime passed
	int m_solverSubStep = 1;
	// Actual iteration
	int m_cpt;
	int m_sectionCount = 0;
	int m_sectionCurrent = 0; 
	btScalar m_minLength;
	bool useLRA = true;
	bool useBending = true;
	bool useGravity = true;
	bool useCollision = true;
	btScalar m_linearMass=1.0;

	vector<btScalar> collisionFonctionPointX;
	vector<btScalar> collisionFonctionPointY;

	MonotonicSpline1D* spline;

	// Node forces members
	bool impulseCompute = true;

	btScalar penetrationMin = 0;
	btScalar penetrationMax  = 0.1;
	btScalar collisionStiffnessMin = 100;
	btScalar collisionStiffnessMax = 10000;

	btScalar collisionViscosity = 10;

	btScalar m_defaultRestLength; 

	btScalar maxAngle = 0;
	btScalar bendingStiffness = 0;

	// disabled collision detection if this movement 
	btScalar m_collisionSleepingThreshold = 0.0;

	// number of iteration step between each iteration of the collision constraint 
	int m_substepDelayCollision = 1;

	// number of iteration of the resolution on multi-collision node
	int m_subIterationCollision = 1;

	// Node forces members
	bool useHydroAero = true;

	float m_collisionMargin = 0;

	void distanceConstraint();
	void distanceConstraintLock(int limMin, int limMax);
	void LRAConstraint();
	void LRAHierachique();
	void DistanceHierachy(int indexStart, int indexEnd);
	void LRAConstraintNode();
	btVector3 fastTrigoPositionCompute(Node* n);

	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	void ResolveConflitZone(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact,btAlignedObjectArray<int>* indexNodeContact);
	void anchorConstraint ();
	
	void solveContact(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btAlignedObjectArray<int>* indexNodeContact);
	void solveContactLimited(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, int limitMin, int limitMax);

	btVector3 ComputeCollisionSphere(btVector3 pos, btCollisionObject* obj, Node* n);
	//int solveContact(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact);


	btVector3 calculateBodyImpulse(btRigidBody* body, btScalar margin, Node* n, btVector3 normal, btVector3 hitPosition);
	btVector3 PositionStartRayCalculation(Node* n, btCollisionObject* obj);

	// Methods for collision
	void setupNodeForCollision(btAlignedObjectArray<int>* indexNodeContact);
	void resetNormalAndHitPosition(btAlignedObjectArray<int>* indexNodeContact);

	void updateContactPos(Node* n, int position, int step);
	bool checkCondition(Node* n, int step);
	btScalar computeCollisionMargin(btCollisionShape* shape);
	btCollisionWorld::ClosestRayResultCallback castRay(btVector3 positionStart, btVector3 positionEnd, NodePairNarrowPhase* contact, btScalar margin);


	void recursiveBroadPhase(BroadPhasePair* obj, Node* n, btCompoundShape* shape, btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btVector3 minLink, btVector3 maxLink, btTransform transform);
	//void recursiveBroadPhase(BroadPhasePair* obj, Node* n, Node* n1, btCompoundShape* shape, btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btVector3 minLink, btVector3 maxLink, btTransform transform);

	void resetManifoldLifeTime();
	void clearManifoldContact();
	void UpdateManifoldBroadphase(btAlignedObjectArray<BroadPhasePair*> broadphasePair);

	void updateNodeDeltaPos(int iteration);

	btScalar getLinkRestLength(int index);

public:
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count,int section_count, const btVector3* x, const btScalar* m);

	void setControlPoint(vector<btScalar> dataX, vector<btScalar> dataY);
	
	CollisionMode collisionMode;
	btScalar WantedDistance = 0;
	btScalar WantedSpeed = 0;
	btScalar forceResponseCoef;

	void updateLength(btScalar dt);

	void Grows(float dt);

	void Shrinks(float dt);

	void updateNodeData();

	enum CableState
	{
		Valid = 0,
		InternalForcesError = 1,
		ExternalForcesError = 2
	};

	CableState cableState = Valid;

	struct CableData
	{
		float radius;
		float tangentDragCoefficient;
		float normalDragCoefficient;
		int startIndex;
		int endIndex;
	};
	static const std::size_t CableDataSize = sizeof(CableData);
	
	struct NodePos
    {
    	double x;
		double y;
		double z;
    };
	static const std::size_t NodePosSize = sizeof(NodePos);
    	
    struct NodeData
    {
    	float velocity_x;
        float velocity_y;
        float velocity_z;
		float volume;
    };
	static const std::size_t NodeDataSize = sizeof(NodeData);

	btCable::CableData* m_cableData;
	btCable::NodeData* m_nodeData;
	btCable::NodePos* m_nodePos;

	struct SectionInfo
	{
		double RestLength;
		int StartNodeIndex;
		int EndNodeIndex;
		int NumberOfNodes;
	};

	btCable::SectionInfo* m_section;

private:


#pragma region Use methods
public:
	
	btScalar getLength();
	btScalar getRestLength();

	btVector3 getTensionAt(int index);

	void bendingConstraintDistance();

	void setUseBending(bool active);
	bool getUseBending();

	void setBendingMaxAngle(btScalar angle);
	btScalar getBendingMaxAngle();

	void setBendingStiffness(btScalar stiffness);
	btScalar getBendingStiffness();

	void setUseLRA(bool active);
	bool getUseLRA();

	void setUseGravity(bool active);
	bool getUseGravity();

	void setUseCollision(bool active);
	bool getUseCollision();

	void setCollisionParameters(int substepDelayCollision, int subIterationCollision, btScalar sleepingThreshold);

    bool getUseHydroAero();
	void setUseHydroAero(bool active);
	
	bool updateCableData(btCable::CableData &cableData);

	void* getCableNodesPos();

	int getCableState();

	void appendNode(const btVector3& x, btScalar m) override;

	void setTotalMass(btScalar mass, bool fromfaces = false) override;

	bool checkCollisionAnchor(Node* n, btCollisionObject* obj);

	void setCollisionMargin(float colMargin);

	float getCollisionMargin();

	void addSection(btScalar rl,int start,int end,int nbNodes);

	void setDefaultRestLength(btScalar rl);
	void setMinLength(btScalar value);

	int getGrowingState();

	void setWantedGrowSpeedAndDistance(btScalar speed, btScalar distance);
	void setLinearMass(btScalar mass);

	void setCollisionStiffness(btScalar stiffnessMin, btScalar stiffnessMax, btScalar distMin, btScalar distMax);
	void setCollisionViscosity(btScalar viscosity);
	void setCollisionResponseActive(bool active);
	void setCollisionMode(int mode);

#pragma endregion
};
#endif  //_BT_CABLE_H
