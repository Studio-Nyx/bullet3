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

///The btCable is a class that inherits from btSoftBody.
///Its purpose is to be able to create a cable/rope with our own method parameters that Bullet does not implement.
class btCable : public btSoftBody
{
private:
	btVector3* impulses;
	// btVector3* springsForces;

	// Differents forces for the cable:
	void addSpringForces(); // other way to add the spring & damping forces for the links
	void addMoorDyn();
	void addForces();

public:

	void predictMotion(btScalar dt) override;
	void solveConstraints() override;
	static btSoftBody::psolver_t getSolver(ePSolver::_ solver);
	static void PSolve_Anchors(btSoftBody* psb, btScalar kst, btScalar ti);
	static void PSolve_Links(btSoftBody* psb, btScalar kst, btScalar ti);
	btCable(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m);

	void removeLink(int index);
	void removeNode(int index);
	void removeAnchor(int index);

	void setRestLenghtLink(int index, btScalar distance);
	btScalar getRestLenghtLink(int index);

	void swapNodes(int index0, int index1);
	void swapAnchors(int index0, int index1);

	btScalar getLength();

	btVector3* getImpulses();
};

#endif  //_BT_CABLE_H