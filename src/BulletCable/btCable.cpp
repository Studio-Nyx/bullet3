#include "btCable.h"
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <fstream>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
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

void btCable::solveConstraints()
{
	int i, ni;
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
		impulses[i] = btVector3(0, 0, 0);
	}
	ofstream myfile("contact.txt");
	string file = "";

	list<int> indexNodeBroadPhase;
	btCollisionObjectArray collisionObjectList = world->getCollisionObjectArray();
	//BroadPhase Per Node
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		bool potentialCollision = false;
		btScalar margin = this->m_collisionShape->getMargin();  // + prb1->getCollisionShape()->getMargin();
		btVector3 minLink = btVector3(0, 0, 0);
		btVector3 maxLink = btVector3(0, 0, 0);
	
		minLink.setX(n.m_x.x() - margin);
		minLink.setY(n.m_x.y() - margin);
		minLink.setZ(n.m_x.z() - margin);

		maxLink.setX(n.m_x.x() + margin);
		maxLink.setY(n.m_x.y() + margin);
		maxLink.setZ(n.m_x.z() + margin);

		
		for (int j = 0; j < collisionObjectList.size() && potentialCollision == false; j++)
		{
			btCollisionObject* colObj = collisionObjectList[j];
			int type = colObj->getInternalType();

			// Check if the object is a softbody; we dont do collisions with softbodies
			if (type == 8)
				continue;

			// Check if the object is a rigidbody which have contact on
			if (!colObj->hasContactResponse())
				continue;

			btVector3 mins;
			btVector3 maxs;

			colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(),
												 mins,
												 maxs);
	
			// Intersect box
			if (minLink.x() <= maxs.x() &&
			maxLink.x() >= mins.x() &&
			minLink.y() <= maxs.y() &&
			maxLink.y() >= mins.y() &&
			minLink.z() <= maxs.z() &&
			maxLink.z() >= mins.z())
			{
				file += "position object#";
				file += to_string(j);
				file += ": ";
				file += to_string(colObj->getWorldTransform().getOrigin().x());
				file += ";";
				file += to_string(colObj->getWorldTransform().getOrigin().y());
				file += ";";
				file += to_string(colObj->getWorldTransform().getOrigin().z());
				file += "\n";	

				file += "Scale: ";
				file += to_string(colObj->getCollisionShape()->getLocalScaling().x());
				file += ";";
				file += to_string(colObj->getCollisionShape()->getLocalScaling().y());
				file += ";";
				file += to_string(colObj->getCollisionShape()->getLocalScaling().z());
				file += "\n";

				file += "Flags: ";
				file += to_string(colObj->getCollisionFlags());
				file += "\n";

				file += "ShapeType: ";
				file += to_string(colObj->getCollisionShape()->getShapeType());
				file += "\n";

				indexNodeBroadPhase.push_back(i);
				potentialCollision = true;
			}
			else
			{
				continue;
			}
		}
	} 
		
	// Positions:
	//		getSolver(btSoftBody::ePSolver::Linear)(this, 1, 0);
	// 		distanceConstraint();
	//		LRAConstraint(1, false);
	//		FollowTheLeader();
	//		FABRIKChain();
	//		LRAConstraint();
	// Anchors:	
	//		getSolver(btSoftBody::ePSolver::Anchors)(this, 1, 0);
	//		pinConstraint();
	// Collisions: 
	//		getSolver(btSoftBody::ePSolver::RContacts)(this, 1, 0);		

	for (int i = 0; i < m_cfg.piterations; ++i)
	{
		SolveAnchors();		
		distanceConstraint();
		// if (useLRA) LRAConstraint();
		if (useBending) bendingConstraintDistance();
		// getSolver(btSoftBody::ePSolver::RContacts)(this, 1, 0);	
		if (useCollision) solveContact(i, indexNodeBroadPhase, file);
	}
	if (useCollision) solveContact(-1, indexNodeBroadPhase, file);

	const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];	
		n.m_v = (n.m_x - n.m_q) * vc;
		n.m_f = btVector3(0, 0, 0);	
	}

	myfile << file;
	myfile.close();

	/*
	for (i = 0, ni = m_anchors.size(); i < ni; ++i)
		std::cout << "impulses[" << i << "]:" << impulses[i].length() << std::endl;
	*/

	/*
	std::cout << "================================" << std::endl;
	for (i = 0, ni = m_anchors.size(); i < ni; ++i)
	{
		std::cout << "impulses[" << i << "]: " << impulses[i].x() << " / " << impulses[i].y() << " /  " << impulses[i].z() << std::endl;
	}
	std::cout << "Length[RestL]: " << getLengthRestlength() << std::endl;
	std::cout << "Length[RealL]: " << getLengthPosition() << std::endl;*/
}

void btCable::solveContact(int step, list<int> broadphaseNode, string& myfile)
{
	// Prepare Conctact Resolution
	std::list<int> listNodeContact;
	std::list<btCollisionWorld::ClosestRayResultCallback> callbackValue;
	btScalar margin = getCollisionShape()->getMargin();
	// Collision detection

	myfile += "Potentiel collision: ";
	myfile += to_string(broadphaseNode.size());
	myfile += "\n";

	std::list<int>::iterator nodeBroadphaseIterator;
	for (nodeBroadphaseIterator = broadphaseNode.begin(); nodeBroadphaseIterator != broadphaseNode.end(); nodeBroadphaseIterator++)
	{
		int i = *nodeBroadphaseIterator;
		//Ignore collision on Anchors
		if (m_nodes[i].m_battach != 0) continue;

		btVector3 posFrom;
	
		if (step == 0)
			posFrom = m_nodes[i].m_q;
		else
			// Last position out of the body known
			posFrom = m_nodes[i].m_splitv;

		btVector3 posTo = m_nodes[i].m_x;
		
		
		// No displacement predicted cause 0 velocity or static node
		if (posFrom == posTo)
			continue;
		else
		{
			btCollisionWorld::ClosestRayResultCallback closestResults(posFrom, posTo);
			world->rayTest(posFrom, posTo, closestResults);

			myfile += "\nSHOOT\n";

			myfile += "closetHitFraction: ";
			myfile += to_string(closestResults.m_closestHitFraction);
			myfile += "\n";
			
			myfile += to_string(closestResults.hasHit());
			myfile += "\n";

			myfile += "posFrom: ";
			myfile += to_string(posFrom.x());
			myfile += ";";
			myfile += to_string(posFrom.y());
			myfile += ";";
			myfile += to_string(posFrom.z());
			myfile += "\n";

			myfile += "posTo: ";
			myfile += to_string(posTo.x());
			myfile += ";";
			myfile += to_string(posTo.y());
			myfile += ";";
			myfile += to_string(posTo.z());
			myfile += "\n";

			myfile += "closetHitFraction: ";
			myfile += to_string(closestResults.m_closestHitFraction);
			myfile += "\n";

			myfile += "Flags: ";
			myfile += to_string(closestResults.m_flags);
			myfile += "\n";

			if (closestResults.hasHit())
			{
				myfile += "HIT\n";
				if (closestResults.m_collisionObject->hasContactResponse())
				{
					listNodeContact.push_back(i);
					callbackValue.push_back(closestResults);
				}		
			}
			myfile += "\n";
		}
	}
	list<int>::iterator itNode;
	list<btCollisionWorld::ClosestRayResultCallback>::iterator itCallback;

	// Iteration on each Node witch have contact
	for (itNode = listNodeContact.begin(), itCallback = callbackValue.begin(); itNode != listNodeContact.end(); itNode++,itCallback++)
	{
		int indexNode = *itNode;
		Node* n = &m_nodes[indexNode];
		btCollisionWorld::ClosestRayResultCallback c = *itCallback;
		btVector3 newPos = c.m_hitPointWorld + c.m_hitNormalWorld * (margin);
		myfile += "hitPointWorld: ";
		myfile += to_string(c.m_hitPointWorld.x());
		myfile += ";";
		myfile += to_string(c.m_hitPointWorld.y());
		myfile += ";";
		myfile += to_string(c.m_hitPointWorld.z());
		myfile += "\n";

		myfile += "hitNormalWorld: ";
		myfile += to_string(c.m_hitNormalWorld.x());
		myfile += ";";
		myfile += to_string(c.m_hitNormalWorld.y());
		myfile += ";";
		myfile += to_string(c.m_hitNormalWorld.z());
		myfile += "\n";

		btVector3 deltaPos = n->m_x - newPos;
		btVector3 changementSpeed = c.m_hitNormalWorld.cross(deltaPos);
		changementSpeed = changementSpeed.cross(c.m_hitNormalWorld);
		n->m_x = newPos; // + changementSpeed*(1 - (m_cfg.kDF * c.m_collisionObject->getFriction()));
		n->m_splitv = n->m_x;
	}
}

void btCable::LRAConstraint()
{
	btScalar distance = 0;
	pin();
	if (m_idxAnchor == 1)
	{
		Node& a = m_nodes[m_nodes.size() - 1];
		for (int i = m_links.size() - 1; i >= 0; --i)
		{
			Link& l = m_links[i];
			Node* b = l.m_n[0];

			distance += l.m_rl;
			if (a.m_x.distance(b->m_x) > distance)
				b->m_x = a.m_x + (b->m_x - a.m_x).normalized() * distance;
		}
	}
	else
	{
		Node& a = m_nodes[0];
		for (int i = 0; i < m_links.size() - 1 ;++i)
		{
			Link& l = m_links[i];
			Node* b = l.m_n[1];

			distance += l.m_rl;
			if (a.m_x.distance(b->m_x) > distance)
				b->m_x = a.m_x + (b->m_x - a.m_x).normalized() * distance;
		}
	}
}

void btCable::LRAConstraint(int level, int idxAnchor)
{
	int stablePointIndex = m_anchors[idxAnchor].m_node->index;
	m_anchors[idxAnchor].m_node->m_x = m_anchors[idxAnchor].m_body->getCenterOfMassPosition() + m_anchors[idxAnchor].m_c1;

	// Apply LRA left Side
	for (int j = stablePointIndex - 1; j >= 0; j--)
	{
		Node& n = m_nodes[j];
		btScalar distMax = getRestLengthLink(stablePointIndex - 1);
		int indexToCheck = j - 1;
		if (indexToCheck >= 0)
		{
			Node& left = m_nodes[indexToCheck];
			btVector3 AB = left.m_x - n.m_x;
			if (AB.length() > distMax)
			{
				btVector3 ABNormalized = AB.normalized();
				btVector3 PosLocal = ABNormalized * distMax;
				btVector3 newpos = n.m_x + PosLocal;
				left.m_x = newpos;
			}
		}
	}

	// Apply LRA right Side
	for (int j = stablePointIndex; j < m_nodes.size() - 1; j++)
	{
		Node& n = m_nodes[j];
		btScalar distMax = getRestLengthLink(stablePointIndex);
		int indexToCheck = j + 1;
		if (indexToCheck < m_nodes.size())
		{
			Node& right = m_nodes[indexToCheck];
			btVector3 AB = right.m_x - n.m_x;
			if (AB.length() > distMax)
			{
				btVector3 ABNormalized = AB.normalized();
				btVector3 PosLocal = ABNormalized * distMax;
				btVector3 newpos = n.m_x + PosLocal;
				right.m_x = newpos;
			}
		}
	}
}

void btCable::UpdateAnchors(const btTransform tr,
							btVector3& jointCol0,
							btVector3& jointCol1)
{
	jointCol1 = tr.getBasis() * jointCol0 + tr.getOrigin();
}

btVector3 ConstrainDistance(btVector3 point, btVector3 anchor, float distance)
{
	return ((point - anchor).normalize() * distance) + anchor;
}

void btCable::FABRIKChain()
{
	if (m_idxAnchor == 0)
		for (int i = m_anchors.size() - 1; i >= 0; --i)
		{
			Anchor& a = m_anchors[i];
			a.m_node->m_x = m_anchors[i].m_body->getCenterOfMassPosition() + a.m_c1;
			int idx = a.m_node->index;

			for (int i = idx + 1; i < m_nodes.size(); i++)
			{
				//Pull the current segment to the previous one
				if (m_nodes[i - 1].m_x.distance(m_nodes[i].m_x) > m_links[i - 1].m_rl)
					m_nodes[i].m_x = ConstrainDistance(m_nodes[i].m_x, m_nodes[i - 1].m_x, m_links[i - 1].m_rl);
			}

			for (int i = idx; i > 0; --i)
			{
				//Pull the current segment to the previous one
				if (m_nodes[i].m_x.distance(m_nodes[i - 1].m_x) > m_links[i - 1].m_rl)
					m_nodes[i - 1].m_x = ConstrainDistance(m_nodes[i - 1].m_x, m_nodes[i].m_x, m_links[i - 1].m_rl);
			}
		}
	else if (m_idxAnchor == 1)
		for (int i = 0; i < m_anchors.size(); ++i)
		{
			Anchor& a = m_anchors[i];
			const btVector3 ra = a.m_body->getWorldTransform().getBasis() * a.m_local;

			a.m_node->m_x = m_anchors[i].m_body->getCenterOfMassPosition() + a.m_c1;
			int idx = a.m_node->index;

			for (int i = idx; i > 0; --i)
			{
				//Pull the current segment to the previous one
				if (m_nodes[i - 1].m_x.distance(m_nodes[i].m_x) > m_links[i - 1].m_rl)
					m_nodes[i - 1].m_x = ConstrainDistance(m_nodes[i - 1].m_x, m_nodes[i].m_x, m_links[i - 1].m_rl);
			}

			for (int i = idx + 1; i < m_nodes.size(); i++)
			{
				//Pull the current segment to the previous one
				if (m_nodes[i].m_x.distance(m_nodes[i - 1].m_x) > m_links[i - 1].m_rl)
					m_nodes[i].m_x = ConstrainDistance(m_nodes[i].m_x, m_nodes[i - 1].m_x, m_links[i - 1].m_rl);
			}
		}
}

void btCable::pin()
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor& a = m_anchors[i];
		Node* n = a.m_node;
		btRigidBody* body = a.m_body;
		btTransform tr = body->getWorldTransform();

		btVector3 wa = body->getCenterOfMassPosition() + a.m_c1;
		n->m_x = wa;
	}
}

void btCable::bendingConstraintDistance()
{
	int size = m_nodes.size();
	for (int i = 1; i < this->m_links.size(); ++i)
	{
		Node* before = m_links[i - 1].m_n[0];  // Node before;
		Node* current = m_links[i].m_n[0];     // Current Node
		Node* after = m_links[i].m_n[1];       // Node After
		btVector3 delta1 = current->m_x - before->m_x;
		btVector3 delta2 = after->m_x - current->m_x;
		float stiffness = this->bendingStiffness;
		float iterationFactor = stiffness * stiffness;

		if (delta1.length() <= 0.001 || delta2.length() <= 0.001) continue;

		btScalar dot = delta1.normalized().dot(delta2.normalized());

		if (dot < -1.0f) dot = -1.0f;
		if (dot > 1.0f) dot = 1.0f;
		btScalar phi = acos(dot);
		auto angleMax = this->maxAngle;
		if (phi > angleMax || angleMax == 0)
			stiffness = stiffness;
		else
			stiffness = phi * stiffness / angleMax;

		// DHat
		{
			btVector3 r = after->m_x - before->m_x;
			btScalar minLength = m_links[i].m_rl / 2;

			btScalar rr = r.length2();
			btScalar d2 = btDot(delta2, r);
			btScalar d1 = btDot(delta1, r);
			btClamp(d2, 0.0, rr);
			btClamp(d1, 0.0, rr);
			btScalar alpha1 = d2 / rr;
			btScalar alpha2 = d1 / rr;

			btVector3 d = alpha1 * before->m_x + alpha2 * after->m_x - current->m_x;
			btScalar dLen = d.length();

			if (dLen < DBL_EPSILON)
			{
				continue;
			}

			btVector3 dNorm = d.normalized();
			btVector3 J1 = alpha1 * dNorm;
			btVector3 J2 = -dNorm;
			btVector3 J3 = alpha2 * dNorm;
			btScalar sum = before->m_im * alpha1 * alpha1 + current->m_im + after->m_im * alpha2 * alpha2;
			if (sum <= DBL_EPSILON)
			{
				continue;
			}
			btScalar C = dLen;
			btScalar mass = 1.0 / sum;

			btScalar impulse = -stiffness * mass * C * iterationFactor;

			before->m_x += (before->m_im * impulse) * J1;
			current->m_x += (current->m_im * impulse) * J2;
			after->m_x += (after->m_im * impulse) * J3;
		}
	}
}

void btCable::bendingConstraintAngle()
{
	int size = m_nodes.size();

	for (int i = 1; i < this->m_links.size(); ++i)
	{
		Node* before = m_links[i - 1].m_n[0];  // Node before;
		Node* current = m_links[i].m_n[0];     // Current Node
		Node* after = m_links[i].m_n[1];       // Node After

		btVector3 p0 = before->m_x;
		btVector3 p1 = after->m_x;
		btVector3 p2 = current->m_x;

		btScalar phiZero = 0;

		float stiffness = 1;

		btVector3 axeTan = (p0 - p2).cross(p1 - p2);
		if (axeTan.length() < DBL_EPSILON)
			continue;
		axeTan = axeTan.normalized();

		btVector3 p3 = p2 + axeTan * min(m_links[i - 1].m_rl, m_links[i].m_rl) * 0.5;

		btVector3 e = p3 - p2;
		btScalar elen = e.length();
		btScalar invElen = 1.0 / elen;

		btVector3 n1 = (p2 - p0).cross(p3 - p0);
		btVector3 n2 = (p3 - p1).cross(p2 - p1);

		n1 /= n1.length2();
		n2 /= n2.length2();

		btVector3 d0 = elen * n1;
		btVector3 d1 = elen * n2;
		btVector3 d2 = (p0 - p3).dot(e) * invElen * n1 + (p1 - p3).dot(e) * invElen * n2;
		btVector3 d3 = (p2 - p0).dot(e) * invElen * n1 + (p2 - p1).dot(e) * invElen * n2;

		n1 = n1.normalized();
		n2 = n2.normalized();

		btScalar dot = n1.dot(n2);

		if (dot < -1.0f) dot = -1.0f;
		if (dot > 1.0f) dot = 1.0f;
		btScalar phi = acos(dot);

		btScalar invMass = current->m_im;

		btScalar lambda = invMass * d0.length2() +
						  invMass * d1.length2() +
						  invMass * d2.length2() +
						  invMass * d3.length2();

		lambda = (phi - phiZero) / lambda * stiffness;

		if (abs(lambda) <= FLT_EPSILON)
			continue;

		if (n1.cross(n2).dot(e) > 0.0f)
			lambda = -lambda;

		before->m_x += -invMass * lambda * d0;
		after->m_x += -invMass * lambda * d1;
		current->m_x += -invMass * lambda * d2;
	}
}

void btCable::distanceConstraint()
{
	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		btVector3 AB = b.m_x - a.m_x;
		btScalar normAB = AB.length();
		btScalar k = m_materials[0]->m_kLST;
		btVector3 errAB = k * (normAB - l.m_rl) * ((1 / normAB) * AB);
		
		btScalar sumInvMass = a.m_im + b.m_im;
		btVector3 deltap1 = btVector3(0, 0, 0);
		btVector3 deltap2 = btVector3(0, 0, 0); 
		if (sumInvMass > 0)
		{
			deltap1 = a.m_im / sumInvMass * (AB.length() - l.m_rl) * AB.normalized();
			deltap2 = b.m_im / sumInvMass * (AB.length() - l.m_rl) * AB.normalized();
		}
		a.m_x += deltap1;
		b.m_x -= deltap2;
	}
}

void btCable::predictMotion(btScalar dt)
{
	int i, ni;

	// Prepare
	m_sst.sdt = dt * m_cfg.timescale;
	m_sst.isdt = 1 / m_sst.sdt;
	m_sst.velmrg = m_sst.sdt * 3;
	m_sst.radmrg = getCollisionShape()->getMargin();
	m_sst.updmrg = m_sst.radmrg * (btScalar)0.25;

	// Forces
	if(useGravity) addVelocity(m_worldInfo->m_gravity * m_sst.sdt);


	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;
		btVector3 deltaV = n.m_f * n.m_im * m_sst.sdt;
		n.m_v += deltaV;
		n.m_v *= (1 - m_cfg.kDP);
		n.m_x += n.m_v * m_sst.sdt;
		n.m_f = btVector3(0, 0, 0);

		positionNodes[i] = n.m_x;
	}

	updateBounds();

	// Clear contacts
	m_rcontacts.resize(0);
	m_scontacts.resize(0);

}

btSoftBody::psolver_t btCable::getSolver(ePSolver::_ solver)
{
	switch (solver)
	{
		case ePSolver::Linear:
			return (&btSoftBody::PSolve_Links);
		case ePSolver::Anchors:
			return (&btSoftBody::PSolve_Anchors);
		case ePSolver::RContacts:
			return (&btSoftBody::PSolve_RContacts);
		case ePSolver::SContacts:
			return (&btSoftBody::PSolve_SContacts);
		default:
		{
		}
	}
	return (0);
}

void btCable::SolveAnchors()
{
	BT_PROFILE("PSolve_Anchors");
	const btScalar dt = m_sst.sdt;
	for (int i = 0, ni = m_anchors.size(); i < ni; ++i)
	{
		const Anchor& a = m_anchors[i];
		const btTransform& t = a.m_body->getWorldTransform();
		Node& n = *a.m_node;
		const btVector3 wa = t * a.m_local;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x);
		const btVector3 impulse = a.m_c0 * vr * a.m_influence * m_cfg.kAHR;
		n.m_x += a.m_c2 * impulse;
		impulses[i] += impulse / dt;
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
					// Place & Grab nodes: Blackhole method
					case 1:
					{
						btScalar k = ((l.m_c1 - len) / (l.m_c0 * (l.m_c1 + len))) * kst;
						if (cable->blackHoleIsActive)
						{
							// BlackHole method (use the node's normal)
							btVector3 dirBlackHole = cable->blackHolePos - a.m_x;
							if (dirBlackHole.length() < 0.5)
							{
								btScalar forceBlackHole = 1 / dirBlackHole.length();
								a.m_x -= dirBlackHole * forceBlackHole * k * 10;
							}
						}
						a.m_x -= (del * (k * a.m_im));
						b.m_x += (del * (k * b.m_im));
					}
					break;
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
	collisionShapeNode = new btCapsuleShape(0,0);

	impulses = new btVector3[2]{btVector3(0, 0, 0)};
	positionNodes = new btVector3[node_count]{btVector3(0, 0, 0)};
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

btCollisionShape* btCable::getCollisionShapeNode() const
{
	return collisionShapeNode;
}

void btCable::setCollisionShapeNode(btCollisionShape* nodeShape)
{
	this->collisionShapeNode = nodeShape;
}

void btCable::setWorldRef(btCollisionWorld* colWorld)
{
	this->world = colWorld;
}

btVector3 btCable::getPositionNode(int index)
{
	return positionNodes[index];
}

btVector3 btCable::getImpulse(int index)
{
	return impulses[index];
}

bool btCable::checkIfCollisionWithWorldArrayPos(int objWorldArrayPos)
{
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

void btCable::setBendingMaxAngle(btScalar angle)
{
	this->maxAngle = angle;
}
btScalar btCable::getBendingMaxAngle()
{
	return maxAngle;
}
void btCable::setBendingStiffness(btScalar stiffness)
{
	this->bendingStiffness = stiffness;
}
btScalar btCable::getBendingStiffness()
{
	return this->bendingStiffness;
}

void btCable::setAnchorIndex(int idx)
{
	m_idxAnchor = idx;
}

int btCable::getAnchorIndex()
{
	return m_idxAnchor;
}

#pragma region Use methods
void btCable::setUseLRA(bool active)
{
	useLRA = active;
}

bool btCable::getUseLRA()
{
	return useLRA;
}

void btCable::setUseBending(bool active)
{
	useBending = active;
}

bool btCable::getUseBending()
{
	return useBending;
}

void btCable::setUseGravity(bool active)
{
	useGravity = active;
}

bool btCable::getUseGravity()
{
	return useGravity;
}

void btCable::setUseCollision(bool active)
{
	useCollision = active;
}

bool btCable::getUseCollision()
{
	return useCollision;
}
#pragma endregion
