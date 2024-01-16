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
	};

	struct NodePairNarrowPhase
	{
		Node* node;
		btVector3 m_Xout;
		Node* node1;
		btVector3 m_Xout1;
		btCollisionShape* collisionShape;
		BroadPhasePair* pair;
		btTransform worldToLocal;

		btVector3 lastPosition;
		btVector3 impulse;
		btVector3 normal;
		btScalar distance;
		bool hit = false;
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
	}

	btAlignedObjectArray<CableManifolds> manifolds;
	

private:
	int m_growingState=0;
	// Number of solverIteration for 1 deltaTime passed
	int m_solverSubStep;
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
	void LRAConstraint();
	void LRAConstraintNode();
	void FABRIKChain();

	void SolveLinkCollision(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact);


	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	void anchorConstraint();
	bool checkCollide(int indexNode);
	void solveContact(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact);

	btVector3 moveBodyCollisionLink(btRigidBody* obj, btScalar margin, btScalar im, btVector3 movement, btScalar penetration, btVector3 normale, btVector3 hitPosition);
	btVector3 moveBodyCollision(btRigidBody* body, btScalar margin, Node* n, btVector3 normale, btVector3 hitPosition);
	btVector3 PositionStartRayCalculation(Node* n, btCollisionObject* obj);
	void recursiveBroadPhase(BroadPhasePair* obj, Node* n, Node* n1, btCompoundShape* shape, btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btVector3 minLink, btVector3 maxLink, btTransform transform);

	void resetManifoldLifeTime();
	void clearManifoldContact(btAlignedObjectArray<BroadPhasePair*> objs);
	void UpdateManifoldBroadphase(btAlignedObjectArray<BroadPhasePair*> broadphasePair);

	btScalar getLinkRestLength(int index);

public:
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count,int section_count, const btVector3* x, const btScalar* m);
	
	btScalar WantedDistance = 0;
	btScalar WantedSpeed = 0;

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
	void bendingConstraintAngle();

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

#pragma endregion
};
#endif  //_BT_CABLE_H
