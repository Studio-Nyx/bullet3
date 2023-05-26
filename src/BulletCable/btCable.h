#ifndef _BT_CABLE_H
#define _BT_CABLE_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btVector3.h"
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

using namespace std;

///The btCable is a class that inherits from btSoftBody.
///Its purpose is to be able to create a cable/rope with our own method parameters that Bullet does not implement.
class btCable : public btSoftBody
{
private:
	btVector3* impulses;
	btVector3* positionNodes;
	btCollisionShape* collisionShapeNode;
	btCollisionWorld* world;
	btPersistentManifold* tempManiforld = new btPersistentManifold();
	btConvexPenetrationDepthSolver* m_pdSolver;
	static void doContact(btSoftBody* psb, btScalar kst);

	// Differents forces for the cable:
	void addForces();

public:
	btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m);

	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	static btSoftBody::psolver_t getSolver(ePSolver::_ solver);
	static void PSolve_Anchors(btSoftBody* psb, btScalar kst, btScalar ti);
	static void PSolve_Links(btSoftBody* psb, btScalar kst, btScalar ti);
	static void PSolve_RContacts(btSoftBody* psb, btScalar kst, btScalar ti);
	bool checkCollide(btCollisionObject* colObjA, btCollisionObject* colObjB, btCollisionWorld::ContactResultCallback& resultCallback) const;
	bool checkContact(const btCollisionObjectWrapper* colObjWrap, const btVector3& x, btScalar margin, btSoftBody::sCti& cti) const override;


	void removeLink(int index);
	void removeNode(int index);
	void removeAnchor(int index);
	void swapNodes(int index0, int index1);
	void swapAnchors(int index0, int index1);

	void setRestLengthLink(int index, btScalar distance);
	btScalar getRestLengthLink(int index);
	btScalar getLengthPosition();
	btScalar getLengthRestlength();
	btVector3* getImpulses();
	btVector3* getPositionNodes();
	btVector3* getPositionNodesArray();
	btCollisionShape* getCollisionShapeNode() const;
	void setCollisionShapeNode(btCollisionShape* nodeShape);
	void setWorldRef(btCollisionWorld* colWorld);

	btVector3 getPositionNode(int index);
	btVector3 getImpulse(int index);
};

#endif  //_BT_CABLE_H
