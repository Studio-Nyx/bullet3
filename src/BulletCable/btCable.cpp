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

btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m)
{
	m_world = world;
	impulses = new btVector3[2]{btVector3(0, 0, 0)};
	m_idxAnchor = 0;
	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		m_nodes[i].m_battach = 0;
		m_nodes[i].index = i;
	}
}

#pragma region Constraints
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

	list<int> indexNodeBroadPhase;
	btCollisionObjectArray collisionObjectList = m_world->getCollisionObjectArray();
	//BroadPhase Per Node
	if (useCollision)
	{
		for (i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			if (m_nodes[i].m_battach != 0) continue;

			bool potentialCollision = false;
			btScalar margin = this->m_collisionShape->getMargin();
			btVector3 minLink = btVector3(0, 0, 0);
			btVector3 maxLink = btVector3(0, 0, 0);

			minLink.setX(n.m_x.x() - margin * 2);
			minLink.setY(n.m_x.y() - margin * 2);
			minLink.setZ(n.m_x.z() - margin * 2);

			maxLink.setX(n.m_x.x() + margin * 2);
			maxLink.setY(n.m_x.y() + margin * 2);
			maxLink.setZ(n.m_x.z() + margin * 2);

			for (int j = 0; j < collisionObjectList.size() && potentialCollision == false; j++)
			{
				btCollisionObject* colObj = collisionObjectList[j];
				int type = colObj->getInternalType();

				// Check if the object is not a softbody
				if (type == 8)
					continue;

				// Check if the object is a rigidbody which have contact on
				if ((!colObj->hasContactResponse()))
					continue;

				btVector3 mins, maxs;
				colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), mins, maxs);

				// Intersect box
				if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
					minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
					minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
				{
					indexNodeBroadPhase.push_back(i);
					potentialCollision = true;
				}
			}
		}
	}


	for (int i = 0; i < m_cfg.piterations; ++i)
	{
		SolveAnchors();
		distanceConstraint();
		if (useLRA) LRAConstraint();
		if (useBending) bendingConstraintDistance();
		if (useCollision) solveContact(i, indexNodeBroadPhase);
	}
	if (useCollision) solveContact(-1, indexNodeBroadPhase);

	const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_v = (n.m_x - n.m_q) * vc;
		n.m_f = btVector3(0, 0, 0);
	}

	indexNodeBroadPhase.clear();
}

void btCable::solveContact(int step, list<int> broadphaseNode)
{
	// Prepare Conctact Resolution
	std::list<int> listNodeContact;
	std::list<btCollisionWorld::ClosestRayResultCallback> callbackValue;
	btScalar margin = getCollisionShape()->getMargin();
	std::list<int>::iterator nodeBroadphaseIterator;
	for (nodeBroadphaseIterator = broadphaseNode.begin(); nodeBroadphaseIterator != broadphaseNode.end(); nodeBroadphaseIterator++)
	{
		int i = *nodeBroadphaseIterator;
		btVector3 posFrom = m_nodes[i].m_q;
		btVector3 posTo = m_nodes[i].m_x;

		// No displacement predicted cause 0 velocity or static node
		if (posFrom == posTo)
			continue;
		else
		{
			btCollisionWorld::ClosestRayResultCallback closestResults(posFrom, posTo);
			m_world->rayTest(posFrom, posTo, closestResults);
			if (closestResults.hasHit())
			{
				if (closestResults.m_collisionObject->hasContactResponse())
				{
					listNodeContact.push_back(i);
					callbackValue.push_back(closestResults);
				}
			}
		}
	}

	list<int>::iterator itNode;
	list<btCollisionWorld::ClosestRayResultCallback>::iterator itCallback;
	// Iteration on each Node witch have contact
	for (itNode = listNodeContact.begin(), itCallback = callbackValue.begin(); itNode != listNodeContact.end(); itNode++, itCallback++)
	{
		int indexNode = *itNode;
		Node* n = &m_nodes[indexNode];
		btCollisionWorld::ClosestRayResultCallback c = *itCallback;
		btVector3 newPos = c.m_hitPointWorld + c.m_hitNormalWorld * (margin * 2 * m_cfg.kKHR);
		btVector3 deltaPos = n->m_x - newPos;
		btVector3 changementSpeed = c.m_hitNormalWorld.cross(deltaPos);
		changementSpeed = changementSpeed.cross(c.m_hitNormalWorld);
		n->m_x = newPos + changementSpeed * (1 - (m_cfg.kDF * c.m_collisionObject->getFriction()));
	}
}

void btCable::LRAConstraint()
{
	btScalar distance = 0;
	if (m_idxAnchor == 1)
	{
		Node& a = m_nodes[m_nodes.size() - 1];
		for (int i = 0; i < m_anchors.size(); ++i)
			if (a.index == m_anchors[i].m_node->index)
				a.m_x = m_anchors[i].m_c1 + m_anchors[i].m_body->getCenterOfMassPosition(); 
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
		for (int i = 0; i < m_anchors.size(); ++i)
			if (a.index == m_anchors[i].m_node->index)
				a.m_x = m_anchors[i].m_c1 + m_anchors[i].m_body->getCenterOfMassPosition(); 
		for (int i = 0; i < m_links.size() - 1; ++i)
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
		if (sumInvMass != 0 && a.m_im != 0 && b.m_im != 0)
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
	if (useGravity) addVelocity(m_worldInfo->m_gravity * m_sst.sdt);

	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;
		btVector3 deltaV = n.m_f * n.m_im * m_sst.sdt;
		n.m_v += deltaV;
		n.m_v *= (1 - m_cfg.kDP);
		n.m_x += n.m_v * m_sst.sdt;
		n.m_f = btVector3(0, 0, 0);
	}

	updateBounds();

	// Clear contacts
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
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
		const btVector3 impulseBase = a.m_c0 * vr * a.m_influence * m_cfg.kAHR;
		n.m_x = a.m_body->getCenterOfMassPosition() + a.m_c1;
		impulses[i] += impulseBase / dt;
		a.m_body->applyImpulse(-impulseBase, a.m_c1);

		// const btVector3 impulseMass = vr * a.m_influence * m_cfg.kAHR;
		// a.m_body->setLinearVelocity(a.m_body->getLinearVelocity() - impulseMass);
		// a.m_body->setAngularVelocity(a.m_body->getAngularVelocity() - a.m_c1.cross(impulseMass));
	}
}
#pragma endregion

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

#pragma region Getter/Setter
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

btVector3 btCable::getImpulse(int index)
{
	switch (index)
	{
		case 0:
			for (int i = 0; i < m_anchors.size(); ++i)
			{
				if (m_anchors[i].m_node->index == 0)
					return impulses[i];
			}
			return btVector3(0, 0, 0);
			break;

		case 1:
			for (int i = 0; i < m_anchors.size(); ++i)
			{
				if (m_anchors[i].m_node->index == (m_nodes.size() - 1))
					return impulses[i];
			}
			return btVector3(0, 0, 0);
			break;

		default:
			return btVector3(0, 0, 0);
			break;
	}

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
