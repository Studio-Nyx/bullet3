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
		impulses[i] = btVector3(0, 0, 0);
	}

	// Positions:
	//		getSolver(btSoftBody::ePSolver::Linear)(this, 1, 0);
	// 		distanceConstraint();
	//		LRAConstraint(1, false);
	//		FollowTheLeader();
	//		FABRIKChain();
	// Anchors:	
	//		getSolver(btSoftBody::ePSolver::Anchors)(this, 1, 0);
	//		pinConstraint();
	// Collisions: 
	//		getSolver(btSoftBody::ePSolver::RContacts)(this, 1, 0);		
	for (int i = 0; i < m_cfg.piterations; i++)
	{		
		SolveAnchors();
		distanceConstraint();
		if (activeLRA) FABRIKChain();
		if (activeBending) bendingConstraintDistance();
		getSolver(btSoftBody::ePSolver::RContacts)(this, 1, 0);	
	}
	pin();

	

	const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_v = (n.m_x - n.m_q) * vc;
		n.m_f = btVector3(0, 0, 0);
	}

	std::cout << "================================" << std::endl;
	for (i = 0, ni = m_anchors.size(); i < ni; ++i)
	{
		std::cout << "impulses[" << i << "]: " << impulses[i].x() << " / " << impulses[i].y() << " /  " << impulses[i].z() << std::endl;
		std::cout << "Velocity[" << m_anchors[i].m_body->getMass() << "]: " << m_anchors[i].m_body->getVelocityInLocalPoint(m_anchors[i].m_c1).length() << std::endl;
	}
	std::cout << "Length[RestL]: " << getLengthRestlength() << std::endl;
	std::cout << "Length[RealL]: " << getLengthPosition() << std::endl;
}

void btCable::initPointConstraint(btVector3* m_positionErrorBias, btVector3* m_totalLambda, btMatrix3x3* m_effectiveMass)
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor a = m_anchors[i];
		btVector3 cPos = (a.m_body->getCenterOfMassPosition() + a.m_c1) - a.m_node->m_x;
		m_effectiveMass[i] = btMatrix3x3::getIdentity() * a.m_body->getMass();
		m_positionErrorBias[i] = cPos;

		// TODO: warm starting
		m_totalLambda[i] = btVector3(0, 0, 0);
	}
}

void btCable::updatePointConstraint(btVector3* m_positionErrorBias, btVector3* m_totalLambda, btMatrix3x3* m_effectiveMass)
{
	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor a = m_anchors[i];
		if (!a.m_body->isActive() || m_idxAnchor == i) continue;

		btVector3 cVel = a.m_body->getVelocityInLocalPoint(a.m_c1);
		btVector3 cPos = (a.m_body->getCenterOfMassPosition() + a.m_c1) - a.m_node->m_x;
		btVector3 jvb = cPos / m_sst.sdt;
		btVector3 lambda = m_effectiveMass[i] * (-jvb);
		m_totalLambda[i] -= m_positionErrorBias[i] / m_sst.sdt * a.m_body->getInvMass();
		// a.m_body->applyImpulse(-cPos * m_sst.sdt, a.m_c1);

		a.m_body->setLinearVelocity(a.m_body->getLinearVelocity() - m_positionErrorBias[i]);
		a.m_body->setAngularVelocity(a.m_body->getAngularVelocity() + a.m_c1.cross(-m_positionErrorBias[i]));
	}
}

void btCable::FollowTheLeader()
{
	pin();
	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];
		Node* na = l.m_n[0];
		Node* nb = l.m_n[1];

		btVector3 deltaX = nb->m_x - na->m_x;
		if (deltaX.length() > l.m_rl)
			nb->m_x = na->m_x + deltaX.normalized() * l.m_rl;
	}
}

void btCable::LRAConstraint(int level, int idxAnchor)
{
	/*
	btScalar rl = m_links[0].m_rl;
	int size = m_nodes.size();
	btVector3* positionUpdate = new btVector3[size];
	int position = positionUpdate->length();
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

	int stablePointIndex = m_anchors[idxAnchor].m_node->index;
	if (stablePointIndex > 0)
	{
		for (int j = stablePointIndex; j > 0; j--)
		{
			Node& n = m_nodes[j];
			for (int l = 0; l < level; l++)
			{
				int temp = pow(2, l);
				btScalar distMax = getRestLengthLink(stablePointIndex - 1) * temp;
				int indexToCheck = j - temp;
				if (indexToCheck >= 0)
				{
					Node& left = m_nodes[indexToCheck];
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
			for (int l = 0; l < level; l++)
			{
				int temp = pow(2, l);
				btScalar distMax = getRestLengthLink(stablePointIndex) * temp;
				int indexToCheck = j + temp;
				if (indexToCheck < m_nodes.size())
				{
					Node& right = m_nodes[indexToCheck];
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
	*/

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
		for (int i = 0; i < 5; i++)
		{
			distanceConstraint();
			n->m_x = wa;
		}
	}
}


void btCable::bendingConstraintDistance()
{
	int size = m_nodes.size();
	/*
	{
		
		btVector3 before = m_anchors[0].m_body->getCenterOfMassPosition();  // Node before;
		Node* current = m_links[0].m_n[0];     // Current Node
		Node* after = m_links[0].m_n[1];       // Node After
		btVector3 delta1 = current->m_x - before;
		btVector3 delta2 = after->m_x - current->m_x;
		float stiffness = this->bendingStiffness;
		float iterationFactor = (1.0 / m_cfg.piterations) * m_cfg.piterations * stiffness * stiffness;

		btScalar dot = delta1.normalized().dot(delta2.normalized());

		if (dot < -1.0f) dot = -1.0f;
		if (dot > 1.0f) dot = 1.0f;
		btScalar phi = acos(dot);
		auto angleMax = this->maxAngle;  // PI/4
		if (phi > angleMax)
			stiffness = stiffness;
		else
			stiffness = phi * stiffness / angleMax;

		// DHat

		{
			btVector3 r = after->m_x - before;
			btScalar minLength = m_links[0].m_rl / 2;

			btScalar rr = r.length2();
			btScalar d2 = btDot(delta2, r);
			btScalar d1 = btDot(delta1, r);
			btClamp(d2, 0.0, rr);
			btClamp(d1, 0.0, rr);
			btScalar alpha1 = d2 / rr;
			btScalar alpha2 = d1 / rr;

			btVector3 d = alpha1 * before + alpha2 * after->m_x - current->m_x;
			btScalar dLen = d.length();

			if (dLen < DBL_EPSILON)
			{
				
			}
			else
			{
				btVector3 dNorm = d.normalized();
				btVector3 J1 = alpha1 * dNorm;
				btVector3 J2 = -dNorm;
				btVector3 J3 = alpha2 * dNorm;
				btScalar sum = current->m_im + after->m_im * alpha2 * alpha2;
				if (sum <= DBL_EPSILON)
				{
					
				}
				else
				{
					btScalar C = dLen;
					btScalar mass = 1.0 / sum;

					btScalar impulse = -stiffness * mass * C * iterationFactor;

					current->m_x += (current->m_im * impulse) * J2;
					after->m_x += (after->m_im * impulse) * J3;
				}
				
			}

			
		}
	
	}*/

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

		//Solve Triangle PBD
		/*
		float stiffness = 0.5;
		// Triangle bending model
		float W = before->m_im + after->m_im + 2.0f * current->m_im;
		float invW = stiffness / W;

		btVector3 somPos = current->m_x + before->m_x + after->m_x;
		btVector3 d = current->m_x - 0.33 * somPos;

		btVector3 db0 = 2.0f * before->m_im * invW * d;
		btVector3 dv = -4.0f * current->m_im * invW * d;
		btVector3 db1 = 2.0f * after->m_im * invW * d;

		before->m_x += db0;
		current->m_x += dv;
		after->m_x += db1; */

		// PBD distance
		/*
		
		Link a = m_links[i - 1];
		Link b = m_links[i];
		btVector3 diag = after->m_x - before->m_x;
		btScalar l = diag.length();
		float sumMass = after->m_im + before->m_im;
		before->m_x -= stiffness * before->m_im / sumMass * (a.m_rl + b.m_rl - l) * diag;
		after->m_x += stiffness * after->m_im / sumMass * (a.m_rl + b.m_rl - l) * diag;
		*/

		/*
		btVector3 normal1 = delta1.cross(delta2).normalized();
		btVector3 normal2 = -normal1;
		auto t =delta1.dot(delta2)/(delta1.norm() * delta2.norm());
		float currentAngle;
		if (t > 1)
			currentAngle = SIMD_PI;
		else if (t < -1)
			currentAngle = SIMD_PI;
		else
			currentAngle = acos(delta1.dot(delta2) / (delta1.norm() * delta2.norm()));
		float constraintValue = currentAngle - SIMD_PI;

		// Calculate gradients of the constraint function
		btVector3 grad1 = (delta1 / delta1.norm() - delta1.dot(delta2) / (delta1.norm() * delta2.norm()) * delta2 / delta2.norm()).normalized();
		btVector3 grad2 = (delta2 / delta2.norm() - delta1.dot(delta2) / (delta1.norm() * delta2.norm()) * delta1 / delta1.norm()).normalized();

		auto temp = grad1.dot(grad2);
		auto denom = (grad1.length2() * before->m_im + 2 * temp * current->m_im + grad2.length2() * after->m_im);
		if (grad1.dot(grad2) < 0.0001) continue;
		// Compute Lagrange multiplier
		float lambda = -0.5 * constraintValue / (grad1.length2() * before->m_im + 2 * (grad1.dot(grad2)) * current->m_im + grad2.length2() * after->m_im);
		// Apply corrections
		before->m_x += lambda * grad1 * before->m_im;
		current->m_x -= lambda * (grad1 + grad2) * current->m_im;
		after->m_x += lambda * grad2 * after->m_im;
		*/
		/*
		btScalar phi0 = SIMD_PI; 
		btScalar d = (after->m_x - before->m_x).length();
		btScalar sumMass = before->m_im + current->m_im + after->m_im;
		btScalar summNorm  = after->m_x.length2() + before->m_x.length2() + current->m_x.length2();
		float massRatio = 3 * current->m_im / sumMass;
		btScalar racine = sqrt((1 - d)*(1-d));
		btScalar angle = btAcos(d);
		btScalar numerator = racine * angle - phi0;
		btVector3 deltaPi = -massRatio * numerator / summNorm * current->m_x;
		*/
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

		//stiffness = 1 - pow((1 - stiffness), this->m_cfg.piterations);

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

btVector3 btCable::VectorByQuaternion(btVector3 v, btQuaternion q)
{
	btVector3 u = btVector3(q.x(), q.y(), q.z());
	return 2. * u.dot(v) * u + (q.w() * q.w() - u.dot(u)) * v + 2. * q.w() * u.cross(v);
}

btQuaternion btCable::ComputeQuaternion(btVector3 v)
{
	btScalar delta = v.norm();
	return btQuaternion(delta * cos(delta / 2), v.x() * sin(delta / 2), v.y() * sin(delta / 2), v.z() * sin(delta / 2)) * (1 / delta);
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
	int size = m_nodes.size();
	float allInvMasses = 0;
	for (int i = 0; i < m_anchors.size(); ++i)
		allInvMasses += m_anchors[i].m_body->getInvMass();

	for (int i = 0; i < m_anchors.size(); ++i)
	{
		Anchor a = m_anchors[i];
		btRigidBody* body = a.m_body;
		Node* n = a.m_node;
		btTransform tr = body->getWorldTransform();
		btQuaternion qa = tr.getRotation();
		btQuaternion qca = btQuaternion(qa.w(), -qa.x(), -qa.y(), -qa.z());

		btVector3 center = body->getCenterOfMassPosition();
		btVector3 ra = a.m_c1;
		btVector3 xa = center + ra;
		btVector3 xb = n->m_x;
		btVector3 deltaX = xb - xa;
		if (deltaX.fuzzyZero()) continue;
		btVector3 deltaXhat = deltaX.normalized();
		n->m_x = xa;

		if (allInvMasses <= 0.0f || body->getActivationState() == DISABLE_SIMULATION)
			continue;

		float lambda = deltaX.length() / (allInvMasses);
		deltaXhat *= -lambda;
		impulses[i] += deltaXhat;
		body->applyImpulse(-deltaXhat, a.m_c1);
		// body->setLinearVelocity(body->getLinearVelocity() + deltaXhat * body->getInvMass());
		// body->setAngularVelocity(body->getAngularVelocity() + body->getInvInertiaTensorWorld() * a.m_c1.cross(deltaXhat));
		// btTransform predictedTransform;
		// btVector3 linVel = body->getLinearVelocity() + deltaXhat * body->getInvMass();
		// btVector3 angVel = body->getAngularVelocity() + body->getInvInertiaTensorWorld() * a.m_c1.cross(deltaXhat);
		// btTransformUtil::integrateTransform(tr, linVel, angVel, m_sst.sdt, predictedTransform);
		// body->setWorldTransform(predictedTransform);
	}
	/*
	float allInvMasses = 0;
	for (int i = 0; i < m_anchors.size(); ++i)
		if (!m_anchors[i].m_body->getMass() <= 0)
			allInvMasses += m_anchors[i].m_body->getInvMass();

	for (int i = m_anchors.size() - 1; i >= 0; --i)
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

		// 7.5: Velocities
		btVector3 vapin = body->getVelocityInLocalPoint(anchor->m_c1);
		btVector3 vbpin = (a->m_x - a->m_q) * m_sst.isdt;
		btVector3 vAB = vapin - vbpin;
		if (vAB.length() < 1.0e-6f) break;

		// 7.6: Normalized vAB
		btVector3 vbodyA = VectorByQuaternion(vAB.normalized(), qc);

		// 7.7: Compute jv
		btScalar vdotbodyA = vbodyA.dot(body->getInvInertiaTensorWorld() * (ra.cross(vbodyA).cross(ra)));
		btScalar jv = -1 / (body->getInvMass() + a->m_im + vdotbodyA);

		// 7.8: Change the velocities
		body->applyImpulse(jv * vAB + jx * xAB * m_sst.isdt, anchor->m_c1);
	}
	*/
}

/*
	btVector3* positions = new btVector3[m_nodes.size()];

	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		btVector3 displacement = a.m_x - b.m_x;
		btVector3 relativeVelocity = a.m_v - b.m_v;

		// Position
		btVector3 xerr = (l.m_rl - displacement.length()) * displacement.normalized();
		positions[a.index] += (a.m_im / (a.m_im + b.m_im)) * xerr;
		positions[b.index] -= (b.m_im / (a.m_im + b.m_im)) * xerr;

		// Velocity
		btVector3 verr = relativeVelocity.dot(displacement.normalized()) * displacement.normalized();
		a.m_v += (a.m_im / (a.m_im + b.m_im)) * verr;
		b.m_v -= (b.m_im / (a.m_im + b.m_im)) * verr;
	}

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		Node& n = m_nodes[i];
		// n.m_q = n.m_x;
		n.m_x += positions[i];
	}

	delete positions;
	*/

void btCable::distanceConstraint()
{
	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		btVector3 AB = b.m_x - a.m_x;
		btScalar normAB = AB.length();
		btScalar k = 1;
		btVector3 errAB = k * (normAB - l.m_rl) * ((1 / normAB) * AB);

		btScalar sumInvMass = a.m_im + b.m_im;

		btVector3 deltap1 = a.m_im / sumInvMass * (AB.length() - l.m_rl) * AB.normalized();
		btVector3 deltap2 = b.m_im / sumInvMass * (AB.length() - l.m_rl) * AB.normalized();

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
	addVelocity(m_worldInfo->m_gravity * m_sst.sdt);

	// Integrate
	/*
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_q = n.m_x;
		btVector3 deltaV = n.m_f * n.m_im * m_sst.sdt;
		{
			btScalar maxDisplacement = m_worldInfo->m_maxDisplacement;
			btScalar clampDeltaV = maxDisplacement / m_sst.sdt;
			for (int c = 0; c < 3; c++)
			{
				if (deltaV[c] > clampDeltaV)
				{
					deltaV[c] = clampDeltaV;
				}
				if (deltaV[c] < -clampDeltaV)
				{
					deltaV[c] = -clampDeltaV;
				}
			}
		}
		n.m_v += deltaV;
		n.m_x += n.m_v * m_sst.sdt;
		n.m_f = btVector3(0, 0, 0);
	}
	*/

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

	// Nodes
	ATTRIBUTE_ALIGNED16(btDbvtVolume)
	vol;
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

void btCable::AnchorsConstraint(btVector3* jointInfo0, btVector3* jointInfo1)
{
	/*
	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = psb->m_cfg.kAHR * kst;
	const btScalar dt = psb->m_sst.sdt;
	for (int i = 0, ni = psb->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = psb->m_anchors[i];

		btTransform& t = a.m_body->getWorldTransform();
		btVector3 wa = a.m_body->getCenterOfMassPosition() + a.m_c1;
		Node& n = *a.m_node;

		const btVector3 dx = (wa - n.m_x) * kAHR;
		const btVector3 dv = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt - (n.m_x - n.m_q);
		const btVector3 impulse = a.m_c0 * (dv + dx);
		c->impulses[i] += impulse / dt;

		// méthode de base => prb mass/ratio
		//n.m_x += impulse * a.m_c2;
		n.m_x = wa;
		a.m_body->applyImpulse(-impulse , a.m_c1);

		/*
		// Déplace le node sur le body	
		if (c->getAnchorIndex() == i)
		{
			n.m_x = wa;
		}
		// Déplace le body sur le node
		else if (c->getAnchorIndex() != i)
		{
			//a.m_body->applyTorqueImpulse(a.m_c1.cross(-impulse));
			t.setOrigin(n.m_x - a.m_c1);
			a.m_body->setWorldTransform(t);
			//a.m_body->applyImpulse(-impulse * dt, a.m_c1);
			n.m_x = wa;
		}
	*/

	// methode de secours 2 => prb trop de tension
	/*
	for (int i = 0, ni = m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = m_anchors[i];
		btTransform& t = a.m_body->getWorldTransform();
		btVector3 wa = a.m_body->getCenterOfMassPosition() + a.m_c1;
		Node& n = *a.m_node;
		const btVector3 dx = (wa - n.m_x);
		const btVector3 dv = a.m_body->getVelocityInLocalPoint(a.m_c1) * m_sst.sdt - (n.m_x - n.m_q);
		const btVector3 impulse = a.m_body->getMass() * (dx + dv);
		if (m_idxAnchor == 0)
			n.m_x += impulse * a.m_c2;
		else
			n.m_x = wa;
		impulses[i] += impulse / m_sst.sdt;
		a.m_body->applyImpulse(-impulse, a.m_c1);
	}
	*/

	/*
	for (int i = 0; i < m_anchors.size(); i++)
	{
		Anchor a = m_anchors[i];
		btRigidBody* body = a.m_body;
		btScalar invMass0 = body->getInvMass();
		btScalar invMass1 = a.m_node->m_im;
		btVector3 corr_x0 = btVector3(0, 0, 0);
		btQuaternion corr_q0 = btQuaternion(0, 0, 0, 0);
		btVector3 corr_x1 = btVector3(0, 0, 0);
		btTransform tr = body->getWorldTransform();
		btVector3 connector = jointInfo1[i];

		btMatrix3x3 K1, K2;
		K1.setZero();
		K2.setZero();
		K1 = computeMatrix(connector, invMass1, body->getCenterOfMassPosition(), body->getInvInertiaTensorWorld());

		if (invMass1 != 0.0)
		{
			K2[0][0] = invMass1;
			K2[1][1] = invMass1;
			K2[2][2] = invMass1;
		}

		const btVector3 pt = llt(K2 + K1) * (a.m_node->m_x - body->getCenterOfMassPosition() + a.m_c1);

		if (invMass0 != 0.0)
		{
			// Position
			corr_x0 = invMass0 * pt;
			body->applyImpulse(corr_x0, a.m_c1);
			// body->setLinearVelocity(body->getLinearVelocity() + corr_x0 / m_sst.sdt * body->getInvMass());
			//tr.setOrigin(tr.getOrigin() + corr_x0);

			// Rotation
			const btVector3 r0 = connector - body->getWorldTransform().getOrigin();
			const btVector3 ot = (body->getInvInertiaTensorWorld() * (r0.cross(pt)));
			const btQuaternion otQ(ot[0], ot[1], ot[2], 0.0);
			corr_q0.setX(0.5 * (otQ * tr.getRotation()).getX());
			corr_q0.setY(0.5 * (otQ * tr.getRotation()).getY());
			corr_q0.setZ(0.5 * (otQ * tr.getRotation()).getZ());
			tr.setRotation(tr.getRotation() + corr_q0);
			body->setWorldTransform(tr);
		}

		if (invMass1 != 0.0)
		{
			// Particle / Node
			corr_x1 = -invMass1 * pt;
			//a.m_node->m_x += corr_x1;
			const btVector3 ra = tr.getBasis() * a.m_local;
			a.m_node->m_x = m_anchors[i].m_body->getCenterOfMassPosition() + ra;	
		}

		cout << pt[0][0] << " - " << pt[0][1] << " - " << pt[0][2] << endl;
		cout << pt[1][0] << " - " << pt[1][1] << " - " << pt[1][2] << endl;
		cout << pt[2][0] << " - " << pt[2][1] << " - " << pt[2][2] << endl;
	}
	*/
}

btMatrix3x3 btCable::llt(btMatrix3x3 M)
{
	btMatrix3x3 res;
	res.setZero();
	int n = 3;
	for (int i = 0; i < n; ++i)
	{
		for (int k = 0; k < i; ++k)
		{
			btScalar value = M[i][k];
			for (int j = 0; j < k; ++j)
				value -= res[i][j] * res[k][j];
			res[i][k] = value / res[k][k];
		}

		btScalar value = M[i][i];
		for (int j = 0; j < i; ++j)
			value -= res[i][j] * res[i][j];
		res[i][i] = std::sqrt(value);
	}

	return res;  //*res.transpose();
}

btMatrix3x3 btCable::computeMatrix(btVector3 connector, btScalar invMass, btVector3 x0, btMatrix3x3 invInertiaTensorW0)
{
	btMatrix3x3 K1 = btMatrix3x3();
	K1.setZero();
	if (invMass != 0.0)
	{
		// ra: local pos anchor oriented
		btVector3 v = connector - x0;
		btScalar a = v.x();
		btScalar b = v.y();
		btScalar c = v.z();

		// J is symmetric
		const btScalar j11 = invInertiaTensorW0[0][0];
		const btScalar j12 = invInertiaTensorW0[0][1];
		const btScalar j13 = invInertiaTensorW0[0][2];
		const btScalar j22 = invInertiaTensorW0[1][1];
		const btScalar j23 = invInertiaTensorW0[1][2];
		const btScalar j33 = invInertiaTensorW0[2][2];

		K1[0][0] = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + invMass;
		K1[0][1] = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
		K1[0][2] = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
		K1[1][0] = K1[0][1];
		K1[1][1] = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + invMass;
		K1[1][2] = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
		K1[2][0] = K1[0][2];
		K1[2][1] = K1[1][2];
		K1[2][2] = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + invMass;
	}
	return K1;
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
		const btVector3 impulse = a.m_c0 * vr * a.m_influence;
		n.m_x += impulse * a.m_c2;
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

				btVector3 posOnSurface = cti.m_contactManifoldPos + cti.m_normal * mrg;
				btVector3 deltaPos = pos - posOnSurface;
				btScalar dp = btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset;
				const btVector3 fv = vr - (cti.m_normal * dn);

				// c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
				//const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3) + (cti.m_normal * (dp * c.m_c4))) * kst);
				const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3)));

				// Friction correction value
				btVector3 frictionValue = impulse * c.m_c2;
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

btCollisionShape* btCable::getCollisionShapeNode() const
{
	return collisionShapeNode;
}

void btCable::setCollisionShapeNode(btCollisionShape* nodeShape)
{
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
	MyContactResultCallback(float margin) : m_haveContact(false), m_margin(margin), norm(btVector3(0, 0, 0)) {}
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
						   btSoftBody::sCti& cti) const
{
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
	//nodeShape.setCollisionShape(new btCapsuleShapeX(margin, margin));
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

void btCable::setLRAActivationState(bool active)
{
	activeLRA = active;
}

bool btCable::getLRAActivationState()
{
	return activeLRA;
}

void btCable::setBendingActivationState(bool active)
{
	activeBending = active;
}

bool btCable::getBendingActivationState()
{
	return activeBending;
}
