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
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include <list>
#include <vector>

using namespace std;

///The btCable is a class that inherits from btSoftBody.
///Its purpose is to be able to create a cable/rope with our own method parameters that Bullet does not implement.
class btCable : public btSoftBody
{
private:
	int m_idxAnchor;
	btVector3* impulses;
	bool useLRA = true;
	bool useBending = true;
	bool useGravity = true;
	bool useCollision = true;
	btScalar maxAngle = 0.1;
	btScalar bendingStiffness = 0.1;

	void distanceConstraint();
	void LRAConstraint();
	void LRAConstraint(int level, int idxAnchor);
	void FABRIKChain();
	void pin();
	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	void SolveAnchors();
	void solveContact(int step, list<int> broadphaseNode);

public:
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m);

	void removeLink(int index);
	void removeNode(int index);
	void removeAnchor(int index);
	void swapNodes(int index0, int index1);
	void swapAnchors(int index0, int index1);

#pragma region Use methods
public:
	void setRestLengthLink(int index, btScalar distance);
	btScalar getRestLengthLink(int index);

	btScalar getLengthPosition();
	btScalar getLengthRestlength();

	btVector3* getImpulses();
	btVector3 getImpulse(int index);

	void bendingConstraintDistance();
	void bendingConstraintAngle();

	void setUseBending(bool active);
	bool getUseBending();

	void setBendingMaxAngle(btScalar angle);
	btScalar getBendingMaxAngle();

	void setBendingStiffness(btScalar stiffness);
	btScalar getBendingStiffness();

	void setAnchorIndex(int idx);
	int getAnchorIndex();

	void setUseLRA(bool active);
	bool getUseLRA();

	void setUseGravity(bool active);
	bool getUseGravity();

	void setUseCollision(bool active);
	bool getUseCollision();
#pragma endregion
};

#endif  //_BT_CABLE_H
