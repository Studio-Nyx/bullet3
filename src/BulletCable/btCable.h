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
		btCollisionShape* collisionShape;
		BroadPhasePair* pair;
		btTransform worldToLocal;
		btVector3 m_Xout;
		btVector3 lastPosition;
		btVector3 impulse;
		btVector3 normal;
		btScalar distance;
		bool hit = false;
	};

	struct CableManifolds
	{
		btPersistentManifold* manifold;
		int lifeTime;
	};
	

	//
	~btCable()
	{
		int size = manifolds.size();
		for (int i = 0; i < size; i++)
		{
			m_world->getDispatcher()->releaseManifold(manifolds.at(i)->manifold);
		}
		manifolds.clear();
	}

	btAlignedObjectArray<CableManifolds*> manifolds;

	

private:
	// Number of solverIteration for 1 deltaTime passed
	int m_solverSubStep;
	// Actual iteration
	int m_cpt;

	bool useLRA = true;
	bool useBending = true;
	bool useGravity = true;
	bool useCollision = true;
	btScalar maxAngle = 0.1;
	btScalar bendingStiffness = 0.1;

	// The margin add after node placement
	btScalar m_correctionNormal = 0.003;

	// The margin add on the ray start postion
	btScalar m_safeDirectionThreshold = 0.01;

	// disabled collision detection if this movement 
	btScalar m_collisionSleepingThreshold = 0.0;

	// number of iteration step between each iteration of the collision constraint 
	int m_substepDelayCollision = 2;

	// number of iteration of the resolution on multi-collision node
	int m_subIterationCollision = 3;

	// Node forces members
	bool useHydroAero = true;


	void distanceConstraint();
	void LRAConstraint();
	void LRAConstraint(int level, int idxAnchor);
	void FABRIKChain();
	
	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	void anchorConstraint();
	bool checkCollide(int indexNode);
	void solveContact(btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact);
	btVector3 moveBodyCollision(btRigidBody* body, btScalar margin, Node* n, btVector3 normale, btVector3 hitPosition);
	btVector3 PositionStartRayCalculation(Node* n, btCollisionObject* obj);
	void recursiveBroadPhase(BroadPhasePair* obj, Node* n, btCompoundShape* shape, btAlignedObjectArray<NodePairNarrowPhase>* nodePairContact, btVector3 minLink, btVector3 maxLink, btTransform transform);

	void resetManifoldLifeTime();
	void clearManifold(btAlignedObjectArray<BroadPhasePair*> objs, bool init);

public:
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m);
	
	float lenght = 0;

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
		float horizonDrop;
		int startIndex;
		int endIndex;
	};
	static const int CableDataSize = sizeof(CableData);
	
	struct NodePos
    {
    	double x;
		double y;
		double z;
    };
    static const int NodePosSize = sizeof(NodePos);
    	
    struct NodeData
    {
    	float velocity_x;
        float velocity_y;
        float velocity_z;
		float volume;
    };
    static const int NodeDataSize = sizeof(NodeData);
    
    btCable::CableData* m_cableData;
	btCable::NodeData* m_nodeData;
    btCable::NodePos* m_nodePos;

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

	void setCollisionParameters(int substepDelayCollision, int subIterationCollision, btScalar correctionNormal, btScalar safeDirectionThreshold, btScalar sleepingThreshold);

    bool getUseHydroAero();
	void setUseHydroAero(bool active);
    void setHorizonDrop(float value);
	
	bool updateCableData(btCable::CableData &cableData);

	void* getCableNodesPos();

	int getCableState();

	void appendNode(const btVector3& x, btScalar m) override;

	void setTotalMass(btScalar mass, bool fromfaces = false) override;

#pragma endregion
};
#endif  //_BT_CABLE_H
