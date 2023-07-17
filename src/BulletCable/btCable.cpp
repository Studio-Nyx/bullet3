#include "btCable.h"
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <fstream>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <Bullet3Common/b3Logging.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <list>
#include "../../Extras/Serialize/BulletWorldImporter/btWorldImporter.h"

using namespace std::chrono;

void btCable::addForces()
{
	std::ofstream MyFile("addForces.txt");

	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];  // the spring
		Node& na = *l.m_n[0];  // 1st particle
		Node& nb = *l.m_n[1];  // 2nd particle

		btVector3 distAB = nb.m_x - na.m_x;     // distance AB
		btVector3 velAB = nb.m_v - na.m_v;      // velocity AB
		btScalar lenAB = distAB.length();       // length AB
		btVector3 dirAB = distAB.normalized();  // direction AB

		btScalar mr = 1 / (na.m_im + nb.m_im);         // mass reduced
		btScalar k = m_materials[0]->m_kLST;           // spring stiffness
		btScalar kMax = mr / (m_sst.sdt * m_sst.sdt);  // spring stiffness max
		btScalar d = m_materials[0]->m_kVST;           // spring damping
		btScalar dMax = mr / m_sst.sdt;                // spring damping max

		// Forces
		btVector3 sForce = k * kMax * (lenAB - l.m_rl) * dirAB;  // spring force
		if (na.m_battach == 0) na.m_f += sForce;
		if (nb.m_battach == 0) nb.m_f -= sForce;

		btScalar velA = dirAB.dot(na.m_v);
		btScalar velB = dirAB.dot(nb.m_v);
		btVector3 dForce = d * dMax * dirAB.dot(velAB) * dirAB;  // damping force
		// if (na.m_battach == 0) na.m_f += dForce;
		// if (nb.m_battach == 0) nb.m_f -= dForce;

		btVector3 gForce = 0.5 * m_worldInfo->m_gravity * mr;  // gravity force
		if (na.m_battach == 0) na.m_f += gForce;
		if (nb.m_battach == 0) nb.m_f += gForce;

		/*
		MyFile << "Link[" << i << "]: " << std::endl;
		{
			MyFile << "	k (stiffness): " << k << std::endl;
			MyFile << "	kMax (stiffness): " << kMax << std::endl;
			MyFile << "	d (damping): " << d << std::endl;
			MyFile << "	dMax (damping): " << dMax << std::endl;

			MyFile << "	Node A: " << std::endl;
			{
				MyFile << "		Mass inv. (a): " << na.m_im << std::endl;
				MyFile << "		Mass (a): " << mr << std::endl;
				MyFile << "		Pos. (a): (" << na.m_x.getX() << "; " << na.m_x.getY() << "; " << na.m_x.getZ() << ")" << std::endl;
				MyFile << "		Vel. (a): (" << na.m_v.getX() << "; " << na.m_v.getY() << "; " << na.m_v.getZ() << ")" << std::endl;
				MyFile << "		Vel. length (a): " << na.m_v.length() << std::endl;
			}

			MyFile << "	Node B:" << std::endl;
			{
				MyFile << "		Mass inv. (b): " << nb.m_im << std::endl;
				MyFile << "		Mass (b): " << mr << std::endl;
				MyFile << "		Pos. (b): (" << nb.m_x.getX() << "; " << nb.m_x.getY() << "; " << nb.m_x.getZ() << ")" << std::endl;
				MyFile << "		Vel. (b): (" << nb.m_v.getX() << "; " << nb.m_v.getY() << "; " << nb.m_v.getZ() << ")" << std::endl;
				MyFile << "		Vel. length (b): " << nb.m_v.length() << std::endl;
			}

			MyFile << "	Forces:" << std::endl;
			{
				MyFile << "		Spring Force: " << sForce.length() << std::endl;		
				MyFile << "		Spring Force (a): (" << sForce.getX() << "; " << sForce.getY() << "; " << sForce.getZ() << ")" << std::endl;
				MyFile << "		Spring Force (b): (" << -sForce.getX() << "; " << -sForce.getY() << "; " << -sForce.getZ() << ")" << std::endl;

				MyFile << "		Damping Force: " << dForce.length() << std::endl;
				MyFile << "		Damping Force (a): (" << dForce.getX() << "; " << dForce.getY() << "; " << dForce.getZ() << ")" << std::endl;
				MyFile << "		Damping Force (b): (" << -dForce.getX() << "; " << -dForce.getY() << "; " << -dForce.getZ() << ")" << std::endl;

				MyFile << "		Gravity Force: " << gForce.length() << std::endl;
				MyFile << "		Gravity Force (a): (" << -gForce.getX() << "; " << -gForce.getY() << "; " << -gForce.getZ() << ")" << std::endl;
				MyFile << "		Gravity Force (b): (" << -gForce.getX() << "; " << -gForce.getY() << "; " << -gForce.getZ() << ")" << std::endl;
			}

			MyFile << std::endl;
		}
	}


	MyFile << std::endl;
	for (int i = 0; i < m_nodes.size(); ++i)
		MyFile << "Total forces node[" << i << "]:" << m_nodes[i].m_f.length() << std::endl;
	MyFile << std::endl;

	MyFile.close();
		*/
	}
}

struct btBridgedManifoldResult : public btManifoldResult
{
	btCollisionWorld::ContactResultCallback& m_resultCallback;

	btBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap, const btCollisionObjectWrapper* obj1Wrap, btCollisionWorld::ContactResultCallback& resultCallback)
		: btManifoldResult(obj0Wrap, obj1Wrap),
		  m_resultCallback(resultCallback) {}

	virtual void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth)
	{
		bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
		btVector3 pointA = pointInWorld + normalOnBInWorld * depth;
		btVector3 localA;
		btVector3 localB;
		if (isSwapped)
		{
			localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
			localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
		}
		else
		{
			localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
			localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
		}

		btManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
		newPt.m_positionWorldOnA = pointA;
		newPt.m_positionWorldOnB = pointInWorld;

		//BP mod, store contact triangles.
		if (isSwapped)
		{
			newPt.m_partId0 = m_partId1;
			newPt.m_partId1 = m_partId0;
			newPt.m_index0 = m_index1;
			newPt.m_index1 = m_index0;
		}
		else
		{
			newPt.m_partId0 = m_partId0;
			newPt.m_partId1 = m_partId1;
			newPt.m_index0 = m_index0;
			newPt.m_index1 = m_index1;
		}
		
		//experimental feature info, for per-triangle material etc.
		const btCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
		const btCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;

		m_resultCallback.addSingleResult(newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
	}
};

void btCable::solveConstraints()
{
	int i, ni;
	// Set the collision Object List
	collisionObjPos.clear();
	for (int i = 0; i < m_rcontacts.size(); i++)
	{
		//btCollisionObject obj = btCollisionObject( *(m_rcontacts.at(i).m_cti.m_colObj));
		int colObjPos = m_rcontacts.at(i).m_cti.m_colObj->getWorldArrayIndex();
		if (!alreadyHaveContact(colObjPos))
		{
			collisionObjPos.push_back(colObjPos);
		}
	}

	// Prepare links
	for (i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		l.m_c3 = l.m_n[1]->m_q - l.m_n[0]->m_q;
		l.m_c2 = 1 / (l.m_c3.length2() * l.m_c0);
	}

	// Prepare anchors
	for (i = 0, ni = m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = m_anchors[i];
		const btVector3 ra = a.m_body->getWorldTransform().getBasis() * a.m_local;
		a.m_c0 = ImpulseMatrix(m_sst.sdt,
							   a.m_node->m_im,
							   a.m_body->getInvMass(),
							   a.m_body->getInvInertiaTensorWorld(),
							   ra);
		a.m_c1 = ra;
		a.m_c2 = m_sst.sdt * a.m_node->m_im;
		a.m_body->activate();
	}

	// Positions: 
	//		getSolver(btSoftBody::ePSolver::Linear)(this, 1, 0);
	// 		distanceConstraint();
	//		LRAConstraint(1, false);
	// Anchors:	
	//		getSolver(btSoftBody::ePSolver::Anchors)(this, 1, 0);
	//		pinConstraint();
	// Collisions: 
	//		getSolver(btSoftBody::ePSolver::RContacts)(this, 1, 0);
	for (int i = 0; i < m_cfg.piterations; ++i)
	{
		getSolver(btSoftBody::ePSolver::Anchors)(this, 1, 0);
		// pinConstraint();
		// distanceConstraint();
		getSolver(btSoftBody::ePSolver::Linear)(this, 1, 0);
		getSolver(btSoftBody::ePSolver::RContacts)(this, 1, 0);
	}

	const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_v = (n.m_x - n.m_q) * vc;
		n.m_f = btVector3(0, 0, 0);
	}

	// LRAConstraint(1);
	// pin();
}

void btCable::pin() {
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor& a = m_anchors[i];
		Node* n = a.m_node;
		btRigidBody* body = a.m_body;
		btTransform tr = body->getWorldTransform();

		btVector3 wa = body->getCenterOfMassPosition() + a.m_c1;
		n->m_x = wa;
		// tr.setOrigin(n->m_x - wa);
		// a.m_node->m_v = body->getVelocityInLocalPoint(a.m_c1);
	}
}

btVector3 btCable::VectorByQuaternion(btVector3 v, btQuaternion q) {
	btVector3 u = btVector3(q.x(), q.y(), q.z());
	return 2. * u.dot(v) * u + (q.w() * q.w() - u.dot(u)) * v + 2. * q.w() * u.cross(v);
}

btQuaternion btCable::ComputeQuaternion(btVector3 v) 
{
	btScalar delta = v.norm();
	return btQuaternion(delta * cos(delta / 2), v.x() * sin(delta / 2), v.y() * sin(delta / 2), v.z() * sin(delta / 2)) * (1/delta);
}

/*
void btCable::pinConstraint()
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		// 7.0: Parameters
		Anchor* anchor = &m_anchors[i];
		btRigidBody* body = anchor->m_body;
		btTransform tr = body->getWorldTransform();
		btVector3 xa = body->getCenterOfMassPosition();
		btVector3 ra = anchor->m_c1;
		btQuaternion q = body->getOrientation();
		btQuaternion qc = btQuaternion(q.w(), -q.x(), -q.y(), -q.z());
		Node* a = anchor->m_node;

		// 7.1: Positions
		btVector3 xapin = xa + ra;
		btVector3 xbpin = a->m_x;
		btVector3 xAB = xapin - xbpin;
		if (xAB.length() < 1.0e-6f) break;

		// 7.2: Normalized xAB
		btVector3 xbodyA = VectorByQuaternion(xAB.normalized(), qc);

		// 7.3: Compute jx
		btScalar xdotbodyA = xbodyA.dot(body->getInvInertiaTensorWorld() * (ra.cross(xbodyA).cross(ra)));
		btScalar jx = -1 / (body->getInvMass() + a->m_im + xdotbodyA);

		// 7.4: Move the anchors & nodes
		tr.setOrigin(tr.getOrigin() + jx * xAB * body->getInvMass());
		a->m_x -= jx * a->m_im * xAB;

		// 7.5: Velocities
		btVector3 vapin = body->getVelocityInLocalPoint(anchor->m_c1);
		btVector3 vbpin = a->m_v;
		btVector3 vAB = vapin - vbpin;
		if (vAB.length() < 1.0e-6f) break;

		// 7.6: Normalized vAB
		btVector3 vbodyA = VectorByQuaternion(vAB.normalized(), qc); 

		// 7.7: Compute jv
		btScalar vdotbodyA = vbodyA.dot(body->getInvInertiaTensorWorld() * (ra.cross(vbodyA).cross(ra)));
		btScalar jv = -1 / (body->getInvMass() + a->m_im + vdotbodyA);

		// 7.8: Change the velocities
		body->applyImpulse(jv * vAB * body->getInvMass(), anchor->m_c1);
		// a->m_v -= jv * a->m_im * vAB;

		// 7.9: Rotation ?

		// 7.10: Update the body's positions
		body->setWorldTransform(tr);
	}
}
*/

void btCable::pinConstraint()
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		if (m_anchors[i].m_body->getMass() == 0)
		{
			continue;
		}
		// 7.0: Parameters
		Anchor* anchor = &m_anchors[i];
		btRigidBody* body = anchor->m_body;
		btTransform tr = body->getWorldTransform();
		// World pos body
		btVector3 xa = body->getCenterOfMassPosition();
		//local oriented anchor
		btVector3 ra = anchor->m_c1;
		btQuaternion q = body->getOrientation();
		btQuaternion qc = btQuaternion(q.w(), -q.x(), -q.y(), -q.z());
		Node* a = anchor->m_node;
		// 7.1: Positions
		// Anchor world pos
		btVector3 xapin = xa + ra;
		// Anchor node position
		btVector3 xbpin = a->m_x;
		btVector3 deltaPosition = xapin - xbpin;
		if (deltaPosition.length() < 0.001)
			continue;
		// 7.2: Normalized xAB
		btVector3 deltaXNormBodyA = VectorByQuaternion(deltaPosition.normalized(), qc);
		// 7.3: Compute jx
		btScalar xdotbodyA = deltaXNormBodyA.dot(body->getInvInertiaTensorWorld() * ra.cross(ra.cross(deltaXNormBodyA)));
		btScalar jx = -1 / (body->getInvMass() + a->m_im + xdotbodyA);

		btVector3 jaBody = VectorByQuaternion(jx * deltaPosition, qc);
		//auto deltaQa = ComputeQuaternion(q * (body->getInvInertiaTensorWorld()*(ra*jaBody))*qc);

		btQuaternion deltaQuaternion = ComputeQuaternion(VectorByQuaternion(body->getInvInertiaTensorWorld() * ra.cross(jaBody), q));
		//btQuaternion deltaQuaternion = btQuaternion();
		btVector3 vapin = body->getVelocityInLocalPoint(anchor->m_c1);
		btVector3 vbpin = (a->m_x - a->m_q) * m_sst.isdt;  // chekc this
		//btVector3 vbpin = a->m_v;  // check this

		btVector3 vAB = vapin - vbpin;
		//btVector3 vAB = vapin;
		//if (vAB.length() < 1.0e-6f) continue;

		btVector3 vbodyA = VectorByQuaternion(vAB.normalized(), qc);
		btScalar vdotbodyA = vbodyA.dot(body->getInvInertiaTensorWorld() * (ra.cross(vbodyA).cross(ra)));
		//btScalar jv = -1 / ( a->m_im + vdotbodyA);
		btScalar jv = -1 / (body->getInvMass() + a->m_im + vdotbodyA);
		btVector3 deltaVelA = jv * a->m_im * vAB;

		tr.setOrigin(a->m_x - ra);
		const btVector3 wa = tr * anchor->m_local;
		const btScalar kAHR = m_cfg.kAHR;
		const btScalar dt = m_sst.sdt;
		const btVector3 va = anchor->m_body->getVelocityInLocalPoint(anchor->m_c1) * dt;
		a->m_v = anchor->m_body->getVelocityInLocalPoint(anchor->m_c1);
		const btVector3 vb = a->m_x - a->m_q;
		//const btVector3 vb = btVector3 (0,0,0);
		//const btVector3 vb = a->m_v ;
		const btVector3 vr = (va - vb) + (wa - a->m_x) * kAHR;
		const btVector3 impulse = anchor->m_c0 * vr * anchor->m_influence;
		anchor->m_body->applyTorqueImpulse(anchor->m_c1.cross(-impulse));

		body->setWorldTransform(tr);
	}
}

void btCable::distanceConstraint()
{
	for (int i = m_links.size() - 1; i >= 0; --i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		btVector3 AB = b.m_x - a.m_x;
		btScalar normAB = AB.length();
		btScalar k = 1;
		btVector3 errAB = k * (normAB - l.m_rl) * ((1 / normAB) * AB);

		a.m_x += 0.5 * errAB;
		b.m_x -= 0.5 * errAB;
	}
}

/*
void btCable::LRAConstraint(int level, bool stable)
{
	btScalar rl = m_links[0].m_rl;
	int size = m_nodes.size();
	btVector3* di = new btVector3[size];

	// iteration for each level
	for (int l = 0; l <= level; l++)
	{
		int temp = pow(2, l);
		btScalar distMax = rl * temp;

		// We check each node Position
		for (int i = 1; i < size - 1; ++i)
		{
			Node& n = m_nodes[i];
			int indexLeft = i - pow(2, l);
			int indexRight = i + pow(2, l);

			if (indexLeft > level)
			{
				Node& left = m_nodes[indexLeft];
				btVector3 deltaPos = n.m_x - left.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = left.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - n.m_x;
					di[i] += move;
				}
			}

			if (indexRight < size - level)
			{
				Node& right = m_nodes[indexRight];
				btVector3 deltaPos = n.m_x - right.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = right.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - n.m_x;
					di[i] += move;
				}
			}
		}
	}

	if (level > 0)
	{
		for (int i = 0; i < size; i++)
			m_nodes[i].m_x += di[i] / (float)pow(2, level);
	}

	delete[] di;
}
*/

void btCable::LRAConstraint(int level)
{
	btScalar rl = m_links[0].m_rl;
	int size = m_nodes.size();

	int stablePointIndex = 0;
	// choose start body
	if (m_anchors[0].m_body->getMass() != 0)
	{
		btScalar topMass = m_anchors[0].m_body->getMass();
		for (int i = 1; i < m_anchors.size(); i++)
		{
			if (m_anchors[i].m_body->getMass() == 0)
			{
				stablePointIndex = m_anchors[i].m_node->index;
				break;
			}
			else if (m_anchors[i].m_body->getMass() > topMass)
			{
				stablePointIndex = m_anchors[i].m_node->index;
				topMass = m_anchors[i].m_body->getMass();
			}
		}
	}
	//cout << "StablePointIndex : " << stablePointIndex << endl;

	if (stablePointIndex > 0)
	{
		for (int j = stablePointIndex; j > 0; j--)
		{
			Node& n = m_nodes[j];
			for (int l = 0; l <= level; l++)
			{
				int temp = pow(2, l);
				btScalar distMax = rl * temp;
				int indexToCheck = j - temp;
				if (indexToCheck >= 0)
				{
					Node& left = m_nodes[indexToCheck];
					//btVector3 deltaPos = n.m_x - right.m_x;  //right To N
					btVector3 AB = left.m_x - n.m_x;
					btScalar ABLength = AB.length();
					if (AB.length() > distMax)
					{
						btVector3 ABNormalized = AB.normalized();
						btVector3 PosLocal = ABNormalized * distMax;
						btVector3 newpos = n.m_x + PosLocal;
						btVector3 deltaPos = newpos - left.m_x;
						left.m_x = newpos;
					}
				}
			}
		}
	}
	// Apply LRA right Side
	if (stablePointIndex < m_nodes.size() - 1)
	{
		for (int j = stablePointIndex; j < m_nodes.size() - 1; j++)
		{
			Node& n = m_nodes[j];
			for (int l = 0; l <= level; l++)
			{
				int temp = pow(2, l);
				btScalar distMax = rl * temp;
				int indexToCheck = j + temp;
				if (indexToCheck < m_nodes.size())
				{
					Node& right = m_nodes[indexToCheck];
					//btVector3 deltaPos = n.m_x - right.m_x;  //right To N
					btVector3 AB = right.m_x - n.m_x;
					auto ABLength = AB.length();
					if (AB.length() > distMax)
					{
						auto ABNormalized = AB.normalized();
						btVector3 PosLocal = ABNormalized * distMax;
						btVector3 newpos = n.m_x + PosLocal;
						btVector3 deltaPos = newpos - right.m_x;
						right.m_x = newpos;
					}
				}
			}
		}
	}
}

/*
void btCable::LRAConstraint(int level, bool isKinematic)
{
	btScalar rl = m_links[0].m_rl;
	int size = m_nodes.size();
	btVector3* di = new btVector3[size];

	/*
	for(int i = 0; i < m_links.size(); ++i)
	{					
		Link& link = m_links[i];
		Node na = *link.m_n[0];
		Node nb = *link.m_n[1];

		for (int l = 1; l <= level; l++)
		{
			int idxLeftLinkShort = i - l;
			int idxLeftLinkLong = i - l - 1;
			int idxRightLinkShort = i + l;
			int idxRigthLinkLong = i + l + 1;

			// for the node nb
			if (idxLeftLinkShort >= 0) {
				btScalar distMax = 0;
				for(int j = idxLeftLinkShort; j < i; ++j)
					distMax = m_links[j].m_rl;

				Link& link = m_links[idxLeftLinkShort];
				Node lls = *link.m_n[0];

				btVector3 deltaPos = nb.m_x - lls.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = lls.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - nb.m_x;
					di[nb.index] += move;
				}
			}

			// for the node na
			if (idxLeftLinkLong >= 0) {
				btScalar distMax = 0;
				for(int j = idxLeftLinkLong; j < i; ++j)
					distMax = m_links[j].m_rl;

				Link& link = m_links[idxLeftLinkLong];
				Node lll = *link.m_n[0];

				btVector3 deltaPos = na.m_x - lll.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = lll.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - na.m_x;
					di[na.index] += move;
				}
			}

			// for the node na
			if (idxRightLinkShort < m_links.size()) {
				btScalar distMax = 0;
				for(int j = i; j < idxRightLinkShort; ++j)
					distMax = m_links[j].m_rl;

				Link& link = m_links[idxRightLinkShort];
				Node lrs = *link.m_n[1];

				btVector3 deltaPos = na.m_x - lrs.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = lrs.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - na.m_x;
					di[na.index] += move;
				}
			}

			// for the node nb
			if (idxRigthLinkLong < m_links.size()) {
				btScalar distMax = 0;
				for(int j = i; j < idxRigthLinkLong; ++j)
					distMax = m_links[j].m_rl;

				Link& link = m_links[idxRigthLinkLong];
				Node lrl = *link.m_n[1];

				btVector3 deltaPos = nb.m_x - lrl.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = lrl.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - nb.m_x;
					di[nb.index] += move;
				}
			}
	
		}
	}

	/*
	for (int l = 1; l <= level; l++) {
		int temp = pow(2, l);
		// We check each node Position
		for (int i = 0; i < size; ++i)
		{
			Node& n = m_nodes[i];
			int indexLeft = i - pow(2, l);
			if (indexLeft > 0)
			{
				btScalar distMax = 0;
				for(int j = i - l; j < i; ++j)
					distMax += m_links[j].m_rl;

				Node& left = m_nodes[indexLeft];
				btVector3 deltaPos = n.m_x - left.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = left.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - n.m_x;
					di[i] += move;
				}
			}
			
			int indexRight = i + pow(2, l);
			if (indexRight < size)
			{			
				btScalar distMax = 0;
				for(int j = i; j < i + l; ++j)
					distMax += m_links[j].m_rl;

				Node& right = m_nodes[indexRight];
				btVector3 deltaPos = n.m_x - right.m_x;
				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = right.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - n.m_x;
					di[i] += move;
				}
			}
		}
	}

	if (level > 0) {
		for (int i = 0; i < size; i++)
			m_nodes[i].m_x += di[i] / level;
	}

	// iteration for each level
	for (int l = 0; l <= level; l++)
	{
		int temp = pow(2, l);
		// We check each node Position
		for (int i = 0; i < size; ++i)
		{
			Node& n = m_nodes[i];
			int indexLeft = i - pow(2, l);
			int indexRight = i + pow(2, l);
			if (indexLeft >= level)
			{
				Node& left = m_nodes[indexLeft];
				btVector3 deltaPos = n.m_x - left.m_x;

				btScalar distMax = 0;
				for(int j = indexLeft; j < i; ++j)
					distMax += m_links[j].m_rl;

				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = left.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - n.m_x;
					di[i] += move;
				}
			}

			if (indexRight < size - level)
			{
				Node& right = m_nodes[indexRight];
				btVector3 deltaPos = n.m_x - right.m_x;

				btScalar distMax = 0;
				for(int j = i; j < indexRight; ++j)
					distMax += m_links[j].m_rl;

				if (deltaPos.length() > distMax)
				{
					btVector3 newpos = right.m_x + deltaPos.normalized() * distMax;
					btVector3 move = newpos - n.m_x;
					di[i] += move;
				}
			}
		}
	}

	if (level > 0)
		for (int i = 0; i < size; i++)
			m_nodes[i].m_x += di[i] / pow(2, level);
}
*/

void btCable::predictMotion(btScalar dt)
{
	int i, ni;

	// Update
	if (m_bUpdateRtCst)
	{
		m_bUpdateRtCst = false;
		updateConstants();
		m_fdbvt.clear();
		if (m_cfg.collisions & fCollision::VF_SS)
		{
			initializeFaceTree();
		}
	}

	// Prepare
	m_sst.sdt = dt * m_cfg.timescale;
	m_sst.isdt = 1 / m_sst.sdt;
	m_sst.velmrg = m_sst.sdt * 3;
	m_sst.radmrg = getCollisionShape()->getMargin();
	m_sst.updmrg = m_sst.radmrg * (btScalar)0.25;

	// Forces 
	addVelocity(m_worldInfo->m_gravity * m_sst.sdt);

	// Integrate
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;
		btVector3 deltaV = n.m_f * n.m_im * m_sst.sdt;
		n.m_v += deltaV;
		n.m_x += n.m_v * m_sst.sdt;
		// n.m_v *= (1 - m_cfg.kDP);
		n.m_f = btVector3(0, 0, 0);
	}
	
	updateBounds();

	// Nodes
	ATTRIBUTE_ALIGNED16(btDbvtVolume) vol;
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		vol = btDbvtVolume::FromCR(n.m_x, m_sst.radmrg);
		m_ndbvt.update(n.m_leaf,
					   vol,
					   n.m_v * m_sst.velmrg,
					   m_sst.updmrg);
	}
	btCollisionObject* obj = this->world->getCollisionObjectArray()[0];
	// Clear contacts
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	
	// Optimize dbvt's
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
}

btSoftBody::psolver_t btCable::getSolver(ePSolver::_ solver)
{
	switch (solver)
	{
		case ePSolver::Linear:
			return (&btCable::PSolve_Links);
		case ePSolver::Anchors:
			return (&btCable::PSolve_Anchors);
		case ePSolver::RContacts:
			return (&btCable::PSolve_RContacts);
		case ePSolver::SContacts:
			return (&btSoftBody::PSolve_SContacts);
		default:
		{
		}
	}
	return (0);
}

void btCable::PSolve_Anchors(btSoftBody* psb, btScalar kst, btScalar ti)
{
	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = psb->m_cfg.kAHR * kst;
	const btScalar dt = psb->m_sst.sdt;
	for (int i = 0, ni = psb->m_anchors.size(); i < ni; ++i)
	{
		const Anchor& a = psb->m_anchors[i];
		const btTransform& t = a.m_body->getWorldTransform();
		Node& n = *a.m_node;
		const btVector3 wa = t * a.m_local;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x) * kAHR;
		const btVector3 impulse = a.m_c0 * vr * a.m_influence;
		n.m_x += impulse * a.m_c2;
		a.m_body->applyImpulse(-impulse, a.m_c1);
	}
}

void btCable::PSolve_Links(btSoftBody* psb, btScalar kst, btScalar ti)
{
	btCable* cable = (btCable*)psb;
	int method = 0;

	BT_PROFILE("PSolve_Links");

	btScalar distance_min = 0;
	for (int i = 0; i < psb->m_links.size(); ++i)
	{
		Link& l = psb->m_links[i];
		distance_min += l.m_rl;
		if (l.m_c0 > 0 && l.m_c0)
		{
			Node& f = psb->m_nodes[0];
			Node& a = *l.m_n[0];
			Node& b = *l.m_n[1];
			btVector3 del = b.m_x - a.m_x;
			const btScalar len = del.length2();
			if (l.m_c1 + len > SIMD_EPSILON)
			{
				switch ((int)method)
				{
					// Place nodes : Based method
					case 0:
					default:
					{
						btScalar k = ((l.m_c1 - len) / (l.m_c0 * (l.m_c1 + len))) * kst;
						a.m_x -= (del * (k * a.m_im));
						b.m_x += (del * (k * b.m_im));
					}
					break;

					// Place nodes: Strict position (not working)
					/*
					case 1:
					{
						del = a.m_n * l.m_rl;
						btVector3 posA = (del * (k * a.m_im));
						btVector3 posB = (del * (k * b.m_im));
						b.m_x = posB + a.m_x + del;
					}
					break;
					*/

					// Place nodes: Position Based Dynamics method
					case 2:
					{
						// diff distance
						btScalar lenAB = del.length();
						btScalar diff = lenAB - l.m_rl;
						btVector3 n = del / lenAB;

						// mass
						btScalar ma = a.m_im / (a.m_im + b.m_im);  // 0.5
						btScalar mb = b.m_im / (a.m_im + b.m_im);  // 0.5

						// positions
						a.m_x -= ma * diff * n;
						b.m_x += mb * diff * n;

						// Stiffness
						btScalar k = cable->m_materials[0]->m_kLST;
						btScalar kprime = 1 - pow(1 - k, 1 / ti);
						// add the stiffness
					}
					break;

					// Place nodes: Distance constraint method
					case 3:
					{
						btVector3 posA = a.m_x;
						btVector3 posB = b.m_x;
						btVector3 direction = (posB - posA).normalize();
						btScalar abs_p12 = abs((posA - posB).length());
						btScalar c = abs_p12 - l.m_rl;

						a.m_x += direction * c / 2;
						b.m_x -= direction * c / 2;
					}
					break;

					// Place & Grab nodes: Blackhole method
					case 4:
					{
						btScalar k = ((l.m_c1 - len) / (l.m_c0 * (l.m_c1 + len))) * kst;
						if (cable->blackHoleIsActive)
						{
							// BlackHole method (use the node's normal)
							btVector3 dirBlackHole = cable->blackHolePos - a.m_x;
							if (dirBlackHole.length() <0.5)
							{
								btScalar forceBlackHole = 1 / dirBlackHole.length();
								a.m_x -= dirBlackHole * forceBlackHole * k*10;
							}	
						}
						a.m_x -= (del * (k * a.m_im));
						b.m_x += (del * (k * b.m_im));
					}
					break;

					// Place nodes: relaxation method
					case 5:
					{
						btScalar deltaD = del.length() - l.m_rl;
						btVector3 direction = del.normalized();
						btScalar k = cable->m_materials[0]->m_kLST;
						btScalar kprime = 1.0 - pow(1.0 - k, ti);

						a.m_x += k * 0.5 * deltaD * direction;
						b.m_x -= k * 0.5 * deltaD * direction;
					}
					break;

					// Dynamic LRA 
					case 6:
						btVector3 del_fb = b.m_x - f.m_x;
						btScalar distance_fb = del_fb.length();

						if (distance_fb > distance_min)
						{
							// simple (direction first node)
							del_fb.normalize();
							b.m_x = del_fb * distance_min;

							// complex (direction link)
							/*
							btScalar slope = 0;
							if (b.m_x.getX() != a.m_x.getX())
								slope = (b.m_x.getY() - a.m_x.getY()) / (b.m_x.getX() - a.m_x.getX());

							btScalar yInt = 0;
							if (b.m_x.getX() == a.m_x.getX())
								yInt = 0;
							else if (b.m_x.getY() == a.m_x.getY())
								yInt = a.m_x.getY();
							else
								yInt = a.m_x.getY() - slope * a.m_x.getX();

							// Collide
							btScalar a = 1 + slope * slope;
							btScalar b = 2 * (slope * (yInt - f.m_x.getY()) - f.m_x.getX());
							btScalar c = f.m_x.getX() * f.m_x.getX() + (slope * (yInt - f.m_x.getY()) * (slope * (yInt - f.m_x.getY()) - distance_min * distance_min;

							btScalar d = b * b - 4 * a * c;

							btScalar inter0 = (-b + sqrt(d)) / 2*a;
							btScalar inter1 = (-b - sqrt(d)) / 2*a;

							b.m_x = 
							*/
						}
						break;
				}
			}
		}
	}
}

void btCable::PSolve_RContacts(btSoftBody* psb, btScalar kst, btScalar ti)
{
	BT_PROFILE("PSolve_RContacts");
	const btScalar dt = psb->m_sst.sdt;
	const btScalar mrg = psb->getCollisionShape()->getMargin();
	btMultiBodyJacobianData jacobianData;

	/*
	std::vector<RContact> tab = vector<RContact>();
	for(int i = 0 ; i < psb->m_rcontacts.size() ; ++i)
		tab.push_back(psb->m_rcontacts[i]);

	std::for_each(
		tab.begin(),
		tab.end(),
		[dt, mrg, kst](auto&& item)
		{
		}
	*/

	// Loop parallel for the collisions
	#pragma loop(hint_parallel(8))
	for (int i = 0, ni = psb->m_rcontacts.size(); i < ni; ++i)
	{
		const RContact& c = psb->m_rcontacts[i];
		const sCti& cti = c.m_cti;
		if (cti.m_colObj->hasContactResponse() && c.m_node->m_battach == 0)  // Collision with anchor disable
		{
			btVector3 vRb(0, 0, 0);
			btRigidBody* rigidCol = 0;
			btMultiBodyLinkCollider* multibodyLinkCol = 0;
			btScalar* deltaV = NULL;

			if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
				vRb = rigidCol ? rigidCol->getVelocityInLocalPoint(c.m_c1) * dt : btVector3(0, 0, 0);
			}
			else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
			{
				multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
				if (multibodyLinkCol)
				{
					const int ndof = multibodyLinkCol->m_multiBody->getNumDofs() + 6;
					jacobianData.m_jacobians.resize(ndof);
					jacobianData.m_deltaVelocitiesUnitImpulse.resize(ndof);
					btScalar* jac = &jacobianData.m_jacobians[0];

					multibodyLinkCol->m_multiBody->fillContactJacobianMultiDof(multibodyLinkCol->m_link, c.m_node->m_x, cti.m_normal, jac, jacobianData.scratch_r, jacobianData.scratch_v, jacobianData.scratch_m);
					deltaV = &jacobianData.m_deltaVelocitiesUnitImpulse[0];
					multibodyLinkCol->m_multiBody->calcAccelerationDeltasMultiDof(&jacobianData.m_jacobians[0], deltaV, jacobianData.scratch_r, jacobianData.scratch_v);

					btScalar vel = 0.0;
					for (int j = 0; j < ndof; ++j)
					{
						vel += multibodyLinkCol->m_multiBody->getVelocityVector()[j] * jac[j];
					}
					vRb = cti.m_normal * vel * dt;
				}
			}

			const btVector3 vNode = c.m_node->m_x - c.m_node->m_q;
			const btVector3 vr = vNode - vRb;
			const btScalar dn = btDot(vr, cti.m_normal);
			
			if (dn <= SIMD_EPSILON)
			{
				//btVector3 deltaPosition = cti.contactManifold.m_positionWorldOnB;
				//const btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
				btVector3 pos = c.m_node->m_x;
				
				btVector3 posOnSurface = cti.m_contactManifoldPos + cti.m_normal*mrg;
				btVector3 deltaPos = pos - posOnSurface; 
				btScalar dp = btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset;
				const btVector3 fv = vr - (cti.m_normal * dn);

				// c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
				//const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3) + (cti.m_normal * (dp * c.m_c4))) * kst);
				const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3) ));

				// Friction correction value
				btVector3 frictionValue = impulse* c.m_c2;
				// Penetration correction
				btVector3 newposDist = cti.m_normal * dp;
				//btVector3 newposDist = cti.m_normal * dp;
				// Updade factor (friction + penetration correction)
				btVector3 updatePos = frictionValue + newposDist;


				
				c.m_node->m_x -= updatePos;
				
				//c.m_node->m_x -= newposDist;
				//c.m_node->m_x = posOnSurface;
				//c.m_node->m_x -= frictionValue;
				//c.m_node->m_x -= impulse * c.m_c2;
				//btVector3 posPost = c.m_node->m_x;
				

				if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
				{
					if (rigidCol)
						rigidCol->applyImpulse(impulse, c.m_c1);
				}

				else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
				{
					if (multibodyLinkCol)
					{
						double multiplier = 0.5;
						multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof(deltaV, -impulse.length() * multiplier);
					}
				}
			}
		}
	}
}

btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m)
{
	this->world = world;
	tempManiforld = new btPersistentManifold();
	void* mem = btAlignedAlloc(sizeof(btGjkEpaPenetrationDepthSolver), 16);
	m_pdSolver = new (mem) btGjkEpaPenetrationDepthSolver;

	impulses = new btVector3[2]{btVector3(0, 0, 0)};
	positionNodes = new btVector3[node_count]{btVector3(0, 0, 0)};
	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		m_nodes[i].m_battach = 0;
		positionNodes[i] = x[i];
	}
}

void btCable::removeLink(int index)
{
	m_links.removeAtIndex(index);
}

void btCable::removeNode(int index)
{
	m_nodes.removeAtIndex(index);
}

void btCable::removeAnchor(int index)
{
	Anchor a = m_anchors[index];
	a.m_node->m_battach = 0;
	m_anchors.removeAtIndex(index);
}

void btCable::setRestLengthLink(int index, btScalar distance)
{
	Link& l = m_links[index];
	l.m_rl = distance;
	l.m_c1 = l.m_rl * l.m_rl;
}

btScalar btCable::getRestLengthLink(int index)
{
	return m_links[index].m_rl;
}

void btCable::swapNodes(int index0, int index1)
{
	m_nodes.swap(index0, index1);
}

void btCable::swapAnchors(int index0, int index1)
{
	m_anchors.swap(index0, index1);
}

btScalar btCable::getLengthRestlength()
{
	btScalar length = 0;
	for (int i = 0; i < m_links.size(); ++i)
		length += m_links[i].m_rl;
	return length;
}

btScalar btCable::getLengthPosition()
{
	btScalar length = 0;
	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		length += (b.m_x - a.m_x).length();
	}

	return length;
}

btVector3* btCable::getImpulses()
{
	return impulses;
}

btVector3* btCable::getPositionNodes()
{
	return positionNodes;
}

btVector3* btCable::getPositionNodesArray() 
{
	btVector3* positions = new btVector3[m_nodes.size()]{btVector3(0, 0, 0)};
	for (int i = 0; i < m_nodes.size(); ++i)
		positions[i] = m_nodes[i].m_x;

	return positions;
}

btCollisionShape* btCable::getCollisionShapeNode() const{
	return collisionShapeNode;
}

void btCable::setCollisionShapeNode(btCollisionShape* nodeShape) {
	this->collisionShapeNode = nodeShape;
}

struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback
{
	bool m_haveContact;
	btScalar m_margin;
	btScalar m_dist = 10000000.0;
	btVector3 norm;
	btManifoldPoint* manifold;
	bool swap = false;
	MyContactResultCallback(float margin) : m_haveContact(false), m_margin(margin), norm(btVector3(0, 0, 0)){}
	virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
	{
		// Detect a contact between the bodies
		if (cp.getDistance() <= m_margin)
		{
			m_haveContact = true;
			if (cp.getDistance() < m_dist)
			{
				m_dist = cp.getDistance();
				norm = cp.m_normalWorldOnB;
				// Obj A = Autre
				// OBJ B = Node
				manifold = &cp; 
			}
		}
		return 1.f;
	}
};

bool btCable::checkContact(const btCollisionObjectWrapper* colObjWrap,
							  const btVector3& x,
							  btScalar margin,
							  btSoftBody::sCti& cti) const{
	/*
	btVector3 nrm;
	const btCollisionShape* shp = colObjWrap->getCollisionShape();
	const btCollisionObject* obj = colObjWrap->getCollisionObject();
	btCollisionObject colObjRef = btCollisionObject(*obj);

	const btTransform& wtr = colObjWrap->getWorldTransform();

	btScalar dst =
		m_worldInfo->m_sparsesdf.Evaluate(
			wtr.invXform(x),
			shp,
			nrm,
			margin);
	if (dst < 0)
	{
		cti.m_colObj = colObjWrap->getCollisionObject();
		cti.m_normal = wtr.getBasis() * nrm;
		cti.m_offset = -btDot(cti.m_normal, x - cti.m_normal * dst);
		return (true);
	}*/
	//cable->collisionObjectList.add();
	const btCollisionObject* obj = colObjWrap->getCollisionObject();
	btCollisionObject colObjRef = btCollisionObject(*obj);
	MyContactResultCallback result(margin);
	btCollisionObject nodeShape;
	btTransform temp = btTransform();
	temp.setIdentity();
	temp.setOrigin(x);
	nodeShape.setWorldTransform(temp);
	nodeShape.setCollisionShape(new btSphereShape(margin));
	
	bool swap = checkCollide(&nodeShape, &colObjRef, result);
	if (result.m_haveContact)
	{		
		cti.m_colObj = colObjWrap->getCollisionObject();
		
		if (swap)
			cti.m_normal = -result.norm;
		else
			cti.m_normal = result.norm;
		
		cti.m_offset = -btDot(cti.m_normal, x - cti.m_normal * result.m_dist);	
		// Contact point On the node shape
		cti.m_contactManifoldPos = result.manifold->getPositionWorldOnB();
		return (true);
	}
	return false;
}
/*
bool btCable::alreadyHaveContact(btCollisionObject obj)const
{
	for (int i = 0; i < collisionList.size(); i++)
	{
		if (collisionList.at(i).getWorldArrayIndex() == obj.getWorldArrayIndex())
		{
			return true;
		}
	}
	return false;
}
*/

bool btCable::alreadyHaveContact(int objPos) const
{
	for (int i = 0; i < collisionObjPos.size(); i++)
	{
		if (collisionObjPos.at(i) == objPos)
		{
			return true;
		}
	}
	return false;
}


void btCable::setWorldRef(btCollisionWorld* colWorld)
{
	this->world = colWorld;
}

bool btCable::checkCollide(btCollisionObject* colObjA, btCollisionObject* colObjB, btCollisionWorld::ContactResultCallback& resultCallback) const 
{	
	btCollisionObjectWrapper obA(0, colObjA->getCollisionShape(), colObjA, colObjA->getWorldTransform(), -1, -1);
	btCollisionObjectWrapper obB(0, colObjB->getCollisionShape(), colObjB, colObjB->getWorldTransform(), -1, -1);
	bool swap = false;
		
	//btCollisionAlgorithm* Tempalgorithm = m_worldInfo->m_dispatcher->findAlgorithm(&obA, &obB, tempManiforld, BT_CLOSEST_POINT_ALGORITHMS);
	btCollisionAlgorithm* Tempalgorithm = m_worldInfo->m_dispatcher->findAlgorithm(&obA, &obB, tempManiforld, BT_CONTACT_POINT_ALGORITHMS);
	
	if (Tempalgorithm)
	{
		swap = Tempalgorithm->isSwapped();
		btBridgedManifoldResult contactPointResult(&obA, &obB, resultCallback);
		contactPointResult.m_closestPointDistanceThreshold = resultCallback.m_closestDistanceThreshold;
		//discrete collision detection query
		Tempalgorithm->processCollision(&obA, &obB, world->getDispatchInfo(), &contactPointResult);
		
		Tempalgorithm->~btCollisionAlgorithm();
		m_worldInfo->m_dispatcher->freeCollisionAlgorithm(Tempalgorithm);
	}
	tempManiforld->clearManifold();
	return swap;
}

btVector3 btCable::getPositionNode(int index) 
{
	return positionNodes[index];
}

btVector3 btCable::getImpulse(int index)
{
	return impulses[index];
}

bool btCable::checkIfCollisionWithWorldArrayPos(int objWorldArrayPos) {
	
	for (int i = 0; i < collisionObjPos.size(); i++)
	{
		if (collisionObjPos.at(i) == objWorldArrayPos)
		{
			return true;
		}
	}
	return false;
}

void btCable::setBlackHolePos(bool activeState, btVector3 pos)
{
	this->blackHolePos = pos;
	this->blackHoleIsActive = activeState;
}

