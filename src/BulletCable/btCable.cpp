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
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>


using namespace std::chrono;


btCable::btCable(btSoftBodyWorldInfo* worldInfo, btCollisionWorld* world, int node_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m)
{
	m_world = world;
	impulses = new btVector3[2]{btVector3(0, 0, 0)};
	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		m_nodes[i].m_battach = 0;
		m_nodes[i].index = i;

		if (i != 0)
		{
			appendLink(i - 1, i);
		}
	}
}


#pragma region Constraints
void btCable::solveConstraints()
{
	int i, ni;
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		m_nodes[i].m_nbCollidingObject = 0;

		// debug draw
		m_nodes[i].m_splitv = btVector3(0, 0, 0);
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

	btAlignedObjectArray<btCollisionObject*> BroadPhaseOutput = btAlignedObjectArray<btCollisionObject*>();
	btAlignedObjectArray<NodePairNarrowPhase> nodePairContact = btAlignedObjectArray<NodePairNarrowPhase>();

	if (useCollision)
	{
		btScalar margin = this->m_collisionShape->getMargin();
		btScalar marginscale = 0.15;
		ni = m_nodes.size();

		// Get the overlapping pairs
		auto cache = m_world->getBroadphase()->getOverlappingPairCache();
		auto temp = cache->getOverlappingPairArray();
		// Maybe a faster way is possible using a filter
		for (int w = 0; w < temp.size(); w++)
		{
			if (temp[w].m_pProxy0->m_clientObject == this)
			{
				// Add Object to the potential list
				btCollisionObject* rb = (btCollisionObject*)temp[w].m_pProxy1->m_clientObject;
				if (rb->hasContactResponse() && this->m_collisionDisabledObjects.findLinearSearch(rb) == m_collisionDisabledObjects.size())
					BroadPhaseOutput.push_back(rb);
			}
			if (temp[w].m_pProxy1->m_clientObject == this)
			{
				// Add the node to the potential list of node collision
				btCollisionObject* rb = (btCollisionObject*)temp[w].m_pProxy0->m_clientObject;
				if (rb->hasContactResponse() && this->m_collisionDisabledObjects.findLinearSearch(rb) == m_collisionDisabledObjects.size())
					BroadPhaseOutput.push_back(rb);
			}
		}
		int potentialCollisionObjectListSize = BroadPhaseOutput.size();

		if (potentialCollisionObjectListSize > 0)
		{
			for (int i = 0; i < ni; i++)
			{
				if (m_nodes[i].m_battach != 0) continue;

				for (int j = 0; j < potentialCollisionObjectListSize; j++)
				{
					btCollisionObject* obj = BroadPhaseOutput[j];
					Node& n = m_nodes[i];
					m_nodes[i].m_xOut = m_nodes[i].m_q;

					// Box Definition
					btVector3 minLink = btVector3(0, 0, 0);
					btVector3 maxLink = btVector3(0, 0, 0);

					minLink.setX(btMin(n.m_x.x() - margin - marginscale, n.m_q.x() - margin - marginscale));
					minLink.setY(btMin(n.m_x.y() - margin - marginscale, n.m_q.y() - margin - marginscale));
					minLink.setZ(btMin(n.m_x.z() - margin - marginscale, n.m_q.z() - margin - marginscale));

					maxLink.setX(btMax(n.m_x.x() + margin + marginscale, n.m_q.x() + margin + marginscale));
					maxLink.setY(btMax(n.m_x.y() + margin + marginscale, n.m_q.y() + margin + marginscale));
					maxLink.setZ(btMax(n.m_x.z() + margin + marginscale, n.m_q.z() + margin + marginscale));

					btVector3 mins, maxs;

					if (obj->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
					{
						btTransform WorldToLocalMatrix = btTransform(obj->getWorldTransform());

						btCompoundShape* temp = (btCompoundShape*)obj->getCollisionShape();
						recursiveBroadPhase(obj, &n, temp, &nodePairContact, minLink, maxLink, WorldToLocalMatrix);
					}
					else
					{
						obj->getCollisionShape()->getAabb(obj->getWorldTransform(), mins, maxs);
						// Intersect box
						if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
							minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
							minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
						{
							auto temp = NodePairNarrowPhase();
							temp.body = BroadPhaseOutput[j];
							temp.node = &n;
							temp.node->m_nbCollidingObject++;
							temp.m_Xout = PositionStartRayCalculation(&n, BroadPhaseOutput[j]);
							temp.collisionShape = obj->getCollisionShape();
							nodePairContact.push_back(temp);
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < m_cfg.piterations; ++i)
	{
		anchorConstraint();
		distanceConstraint(); 
		if (useLRA) LRAConstraint();
		if (useBending && i%2 == 0) bendingConstraintDistance();
		if (useCollision && i%2 == 0) solveContact(nodePairContact);
	}
	
	if (useCollision) solveContact(nodePairContact);

	BroadPhaseOutput.clear();
	nodePairContact.clear();

	const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		n.m_v = (n.m_x - n.m_q) * vc;
		n.m_f = btVector3(0, 0, 0);
	}

	
	/*
	btVector3 positionStartRay = btVector3(1.101, 0, 0);
	btVector3 positionEndRay = btVector3(0, 0, 0);

	btCollisionObject* obj = m_world->getCollisionObjectArray().at(1);

	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;

	m_rayFromTrans.setIdentity();
	m_rayFromTrans.setOrigin(positionStartRay);
	m_rayToTrans.setIdentity();
	m_rayToTrans.setOrigin(positionEndRay);

	btCollisionWorld::ClosestRayResultCallback m_resultCallback(positionStartRay, positionEndRay);
	m_world->rayTestSingleWithMargin(m_rayFromTrans, m_rayToTrans,
									 obj,
									 obj->getCollisionShape(),
									 obj->getWorldTransform(),
									 m_resultCallback,0.1);

	if (m_resultCallback.hasHit())
	{
		cout << "touche" << endl;
	} */
}

void btCable::recursiveBroadPhase(btCollisionObject* obj,Node *n , btCompoundShape* shape, btAlignedObjectArray<NodePairNarrowPhase> *nodePairContact, btVector3 minLink, btVector3 maxLink,btTransform transformLocal)
{
	int subShapes = shape->getNumChildShapes();
	for (int i = 0; i < subShapes; i++) {
		btCollisionShape* temp = shape->getChildShape(i);
		if (temp->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
		{
			btCompoundShape* compound = (btCompoundShape*)temp;
			btTransform newTransform = btTransform();
			newTransform.mult(shape->getChildTransform(i), transformLocal);
			recursiveBroadPhase(obj,n,compound, nodePairContact,minLink,maxLink,newTransform);
		}
		else
		{
			btVector3 mins, maxs;
			auto transform = shape->getChildTransform(i);
			transform.mult(transformLocal, transform);
			temp->getAabb(transform, mins, maxs);
			// Intersect box
			if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
				minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
				minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
			{
				
				auto nodePair = NodePairNarrowPhase();
				nodePair.worldToLocal = transform;
				nodePair.body = obj;
				nodePair.collisionShape = temp;
				nodePair.node = n;
				nodePair.node->m_nbCollidingObject++;
				nodePair.m_Xout = PositionStartRayCalculation(n, obj);
				nodePairContact->push_back(nodePair);
				// debug
			}
		}
	}
}


bool btCable::checkCollide(int indexNode)
{
	
	btCollisionObjectArray array = m_world->getCollisionObjectArray();
	Node n = this->m_nodes[indexNode];
	btScalar margin = this->getCollisionShape()->getMargin();
	for (int i = 0; i < array.size(); i++)
	{
		btVector3 minLink = btVector3(0, 0, 0);
		btVector3 maxLink = btVector3(0, 0, 0);

		minLink.setX(n.m_x.x() - margin);
		minLink.setY(n.m_x.y() - margin);
		minLink.setZ(n.m_x.z() - margin);
		
		maxLink.setX(n.m_x.x() + margin);
		maxLink.setY(n.m_x.y() + margin);
		maxLink.setZ(n.m_x.z() + margin);

		btCollisionObject* colObj = array[i];
		int type = colObj->getInternalType();

		// Check if the object is not a softbody
		if (type == 8)
			continue;

		// Check if the object is a rigidbody which havs contacts
		if ((!colObj->hasContactResponse()))
			continue;

		// Cancel detection with ignored body
		if (m_collisionDisabledObjects.findLinearSearch(colObj) != m_collisionDisabledObjects.size())
			continue;

		btVector3 mins, maxs;
		colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), mins, maxs);

		// Intersect box
		if (minLink.x() <= maxs.x() && maxLink.x() >= mins.x() &&
			minLink.y() <= maxs.y() && maxLink.y() >= mins.y() &&
			minLink.z() <= maxs.z() && maxLink.z() >= mins.z())
		{
			return true;
		}
		
	}
	return false;
}

// Collision between links
// Only need to check on the first iteration
// After that phase nodes will be out of the body and normal collision are enough
void btCable::solveContactLink(btAlignedObjectArray<int> broadphaseNodeList)
{
	const btScalar dt = m_sst.sdt;
	btScalar margin = getCollisionShape()->getMargin();
	for (int nodeIndex = 0; nodeIndex < broadphaseNodeList.size();nodeIndex++)
	{
		int indexNode = broadphaseNodeList.at(nodeIndex);
		btVector3 posNodeBefore = m_nodes[indexNode - 1].m_x;
		btVector3 posNode = m_nodes[indexNode].m_x;
		btVector3 posNodeQ = m_nodes[indexNode].m_q;
		btVector3 dirNodeBeforeToNodeCurrent = (posNode - posNodeBefore).normalized();
		

		// All ray possible
		tVector3Array vectorPossible = tVector3Array();
		{
			vectorPossible.push_back(posNode);
			vectorPossible.push_back(posNode + btVector3(0, margin, 0));
			vectorPossible.push_back(posNode + btVector3(0, -margin, 0));
			vectorPossible.push_back(posNode + btVector3(margin, 0, 0));
			vectorPossible.push_back(posNode + btVector3(-margin, 0, 0));
			vectorPossible.push_back(posNode + btVector3(0, 0, margin));
			vectorPossible.push_back(posNode + btVector3(0, 0, -margin));
		}
		

		std::list<btCollisionWorld::AllHitsRayResultCallback> callbackValue;
		bool collide = false;
		int touch;
		
		// Test every limits around the node 
		for (int i = 0; i < 7; i++)
		{
			// If the first ray didn t hit verify that the node is in the body bounding box
			if (i == 1) 
				if(!checkCollide(indexNode)) 
					break;
			btVector3 posRay = vectorPossible.at(i);
			btCollisionWorld::AllHitsRayResultCallback rayLink(posNodeBefore, posRay);
			m_world->rayTest(posNodeBefore, posRay, rayLink);
			if (rayLink.hasHit())
			{
				for (int j = 0; j < rayLink.m_hitPointWorld.size(); j++)
				{
					// Check if the collision are allowed
					if (rayLink.m_collisionObjects[j]->hasContactResponse() && m_collisionDisabledObjects.findLinearSearch(rayLink.m_collisionObject) == m_collisionDisabledObjects.size())
					{
						collide = true;
						callbackValue.push_back(rayLink);
						touch = j;
						break;
					}
				}	
			}	
			// If 1 intersection is found we can stop
			if (collide) break;
		}

		// If a valid collision is detected
		if (collide)
		{
			btCollisionWorld::AllHitsRayResultCallback ray(callbackValue.back());
			// Get the body and the hitpoint on the touching ray
			btVector3 firstTouch = callbackValue.back().m_hitPointWorld[touch];
			const btCollisionObject* firstObjectTouched = ray.m_collisionObjects[touch];		
			btVector3 velocityObject = firstObjectTouched->getInterpolationLinearVelocity();
			btRigidBody* rigidCol;

			// Get the body velocity
			if (firstObjectTouched->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				rigidCol = (btRigidBody*)btRigidBody::upcast(firstObjectTouched);
				velocityObject = rigidCol->getVelocityInLocalPoint(firstTouch - rigidCol->getWorldTransform().getOrigin());
			}
			btScalar v = velocityObject.length();


			if ( v > FLT_EPSILON)
			{
				btVector3 velocityObjectNormalized = velocityObject.normalized();
				

				
				if ((posNode - posNodeQ).length() > FLT_EPSILON)
				{
					btCollisionWorld::AllHitsRayResultCallback deltaPositionOfTheNodeRay(posNodeQ, posNode);
					m_world->rayTest(posNodeQ, posNode, deltaPositionOfTheNodeRay);
					bool valid = false;
					if (deltaPositionOfTheNodeRay.hasHit())
					{
						for (int size = 0; size < deltaPositionOfTheNodeRay.m_hitPointWorld.size(); size++)
							if (deltaPositionOfTheNodeRay.m_collisionObjects[size]->getWorldArrayIndex() == firstObjectTouched->getWorldArrayIndex())
							{
								valid = true;
								m_nodes[indexNode].m_x = deltaPositionOfTheNodeRay.m_hitPointWorld[size] + deltaPositionOfTheNodeRay.m_hitNormalWorld[size] * (margin);
								m_nodes[indexNode].m_xOut = deltaPositionOfTheNodeRay.m_hitPointWorld[size] + deltaPositionOfTheNodeRay.m_hitNormalWorld[size] * (margin);
								break;
							}
						if (valid) continue;	
					}
				}

				// Throw a ray in the body direction to find if a face push the node or if its a normal collision
				
				btVector3 mouvementDisplacement = (velocityObject * dt) + velocityObjectNormalized * margin;
				btVector3 newPosFrom = m_nodes[indexNode].m_q + mouvementDisplacement;
				btVector3 newPosTo = m_nodes[indexNode].m_q - velocityObjectNormalized * margin;
				

				btCollisionWorld::AllHitsRayResultCallback outRay(newPosFrom, newPosTo);

				// Find the correct position out of the body In the velocity side
				m_world->rayTest(newPosFrom, newPosTo, outRay);
				// If move by body mouvements
				if (outRay.hasHit() )
				{
					for (int size = 0; size < outRay.m_hitPointWorld.size(); size++)
					{
						// Check to find the correct body

						if (outRay.m_collisionObjects[size]->getWorldArrayIndex() == firstObjectTouched->getWorldArrayIndex())
						{
							m_nodes[indexNode].m_x = outRay.m_hitPointWorld[size] + margin * outRay.m_hitNormalWorld[size];
							m_nodes[indexNode].m_xOut = outRay.m_hitPointWorld[size] + margin * outRay.m_hitNormalWorld[size];  
							break;
						}
					}
				}
				
				
				// Case that the body mouvement isn t enought
				else
				{
					btVector3 normale = ray.m_hitNormalWorld[touch];
					btScalar hitFraction = ray.m_hitFractions[touch];
					btVector3 hitPosition = ray.m_hitPointWorld[touch];

					btScalar marginSafe = 0;

					// Vecteur du lien entre previous et Node position avec mouvement du body
					btVector3 rayVector = posNodeBefore - posNode;
					btScalar distanceRay = rayVector.length();

					// Vecteur du lien projeté sur la normale
					btVector3 debugPoint = rayVector.dot(normale) * normale;
					
					btVector3 newFromTest;
					if (debugPoint.length() > FLT_EPSILON)
					{
						btVector3 positionOut = posNode + debugPoint.normalized() * distanceRay;
						btScalar dist = (posNodeBefore - positionOut).length();
						btScalar distLeft = distanceRay - dist;
						
						if (distLeft > FLT_EPSILON)
						{
							btVector3 tangentDir = (positionOut - hitPosition).normalized();
							// Nouveau point hors du body apres application de la velocity
							newFromTest = positionOut;  
							newFromTest += (posNode - newFromTest).normalized() * marginSafe;
						}
						else
						{
							newFromTest = positionOut;
						}
					}
					else
						newFromTest = posNodeBefore;

					// New Ray Test //
					tVector3Array potentialPos;
					tVector3Array potentialNormal;

					// Check the normal side
					int indexHitNormal = -1;
					btCollisionWorld::AllHitsRayResultCallback RayOutObjectNormaleSide(newFromTest, posNode);  
					{
						// Find the correct position out of the body
						m_world->rayTest(newFromTest, posNode,RayOutObjectNormaleSide);
						// Ray in the body link side
						if (RayOutObjectNormaleSide.hasHit())
						{
							for (int size = 0; size < RayOutObjectNormaleSide.m_hitPointWorld.size(); size++)
							{
								if (RayOutObjectNormaleSide.m_collisionObjects[size]->getWorldArrayIndex() == firstObjectTouched->getWorldArrayIndex())
								{
									
									indexHitNormal = size;
									break;
								}
							}
						}
						if (indexHitNormal >= 0)
						{
							btVector3 place = RayOutObjectNormaleSide.m_hitPointWorld[indexHitNormal] + margin * RayOutObjectNormaleSide.m_hitNormalWorld[indexHitNormal];
							
							potentialPos.push_back(place);
							potentialNormal.push_back(RayOutObjectNormaleSide.m_hitNormalWorld[indexHitNormal]);
						}
					}

					// Check the VelocitySide
					int indexHitVelocity = -1;
					btVector3 veloPoint = posNode + (velocityObject * dt) + (velocityObject.normalized() * margin) + (velocityObject.normalized()*marginSafe);
					btCollisionWorld::AllHitsRayResultCallback RayOutVelocitySide(veloPoint, posNode - velocityObject.normalized() * margin);
					{
						m_world->rayTest(veloPoint, posNode - velocityObject.normalized() * margin, RayOutVelocitySide);

						if (RayOutVelocitySide.hasHit())
						{
							for (int size = 0; size < RayOutVelocitySide.m_hitPointWorld.size(); size++)
							{
								if (RayOutVelocitySide.m_collisionObjects[size]->getWorldArrayIndex() == firstObjectTouched->getWorldArrayIndex())
								{
									indexHitVelocity = size;
									break;
								}
							}
						}

						if (indexHitVelocity >= 0)
						{
							btVector3 potential = RayOutVelocitySide.m_hitPointWorld[indexHitVelocity] + margin * RayOutVelocitySide.m_hitNormalWorld[indexHitVelocity];
							potentialPos.push_back(potential);
							potentialNormal.push_back(RayOutVelocitySide.m_hitNormalWorld[indexHitVelocity]);
						}
					}

					// Check the Opposite velocity side
					int indexHitVelocityBack = -1;
					btVector3 oppositePoint = posNode - velocityObject * dt - velocityObject.normalized() * marginSafe;
					btCollisionWorld::AllHitsRayResultCallback RayOutOppositeVelocitySide(oppositePoint, posNode + velocityObject.normalized() * margin);
					{
						m_world->rayTest(oppositePoint, posNode + velocityObject.normalized() * margin, RayOutOppositeVelocitySide);

						if (RayOutOppositeVelocitySide.hasHit())
						{
							for (int size = 0; size < RayOutOppositeVelocitySide.m_hitPointWorld.size(); size++)
							{
								if (RayOutOppositeVelocitySide.m_collisionObjects[size]->getWorldArrayIndex() == firstObjectTouched->getWorldArrayIndex())
								{
									indexHitVelocityBack = size;
									break;
								}
							}
						}
						if (indexHitVelocityBack >= 0)
						{
							btVector3 potential = RayOutOppositeVelocitySide.m_hitPointWorld[indexHitVelocityBack] + margin * RayOutOppositeVelocitySide.m_hitNormalWorld[indexHitVelocityBack];
							potentialPos.push_back(potential);
							potentialNormal.push_back(RayOutOppositeVelocitySide.m_hitNormalWorld[indexHitVelocityBack]);
						}
					}

					
					if (potentialPos.size() == 0 )
					{
						continue;
					}
					// If only one point is detected, set the node at this place
					else
					{

						if (potentialPos.size() == 1)
						{
							for (int elem = 0; elem < potentialPos.size(); elem++)
							{
								m_nodes[indexNode].m_x = potentialPos[elem];
								m_nodes[indexNode].m_xOut = potentialPos[elem];
							}
						}
						// Find the closest one
						else
						{
							int min;
							btScalar minDist = FLT_MAX;
							for (int elem = 0; elem < potentialPos.size(); elem++)
							{
								if ((posNode - potentialPos[elem]).length() <= minDist)
								{
									minDist = (posNode - potentialPos[elem]).length();
									min = elem;
								}
							}
							m_nodes[indexNode].m_x = potentialPos[min];
							m_nodes[indexNode].m_xOut = potentialPos[min];
						}
					}
				}
			}	
			
		}
	}
}

// todo compute Velocity to move mq out of the box
btVector3 btCable::PositionStartRayCalculation(Node *n, btCollisionObject * obj)
{
	btScalar dt = this->m_sst.sdt;
	btVector3 velocity;
	if (obj->getInternalType() == CO_RIGID_BODY)
	{
		btRigidBody* rb = (btRigidBody*)obj;
		velocity = rb->getVelocityInLocalPoint(n->m_x - rb->getWorldTransform().getOrigin());
	}
	else
		velocity = obj->getInterpolationLinearVelocity();
	if (btFuzzyZero(velocity.length())) {
		return n->m_q ;
	}
	return (n->m_q + velocity*dt );
	
}

// Resolve iteratively all the contact constraint
// Do nbSubStep times the resolution to valid a good collision
void btCable::solveContact(btAlignedObjectArray<NodePairNarrowPhase> nodePairContact) {

	btScalar correctionNormal = 0.002;
	btScalar safeDirectionThreshold = 0.01;
	btScalar sleepingThreshold = 0.0001;

	int nbSubStep = 3;
	int size = nodePairContact.size();
	btVector3 positionStartRay;
	btVector3 positionEndRay;

	btTransform m_rayFromTrans;
	btTransform m_rayToTrans;
	btScalar marginNode = this->getCollisionShape()->getMargin();
	btScalar marginBody;
	
	btScalar margin;
	Node* n;
	Node* nBefore;
	Node* nAfter;

	btScalar rlLinkBefore;
	btScalar rlLinkAfter;

	btCollisionObject* obj;
	btCollisionShape* shape;
	btTransform t;
	int indexNode;

	for (int j=0;j<nbSubStep;j++)
	{
		for (int i = 0; i < size; i++)
		{

			n = nodePairContact[i].node;
			// If the node only collide 1 body we don t have to update nbSubStep times
			if (n->m_nbCollidingObject == 1 && j>0)
				continue;

			obj = nodePairContact[i].body;
			shape = nodePairContact[i].collisionShape;
			indexNode = n->index;
			bool isSphereShape = shape->getShapeType() == SPHERE_SHAPE_PROXYTYPE;
			
			// SphereShape margin is sphereShape Radius, not a safe margin
			if (isSphereShape)
			{
				marginBody = 0.00;
				margin = marginNode;  //+ marginBody;
			}
			else
			{
				marginBody = shape->getMargin();
				margin = marginNode + shape->getMargin(); 
			}

			positionStartRay = nodePairContact[i].m_Xout;
			positionEndRay = n->m_x;

			btVector3 dir = btVector3(0, 0, 0);

			btScalar len = (positionEndRay - positionStartRay).length();

			if (btFuzzyZero(len))
				continue;
			if (len <= sleepingThreshold)
			{
				//n->m_x = nodePairContact[i].m_Xout;
				continue;
			}
				
			dir = (positionEndRay - positionStartRay) / len;
			
			
			
			btVector3 start = positionStartRay - dir * safeDirectionThreshold;

			btVector3 end = positionEndRay;  

			m_rayFromTrans.setIdentity();
			m_rayFromTrans.setOrigin(start);

			m_rayToTrans.setIdentity();
			m_rayToTrans.setOrigin(end);

			btCollisionWorld::ClosestRayResultCallback m_resultCallback(start, end);

			// If the body is not a compound it s the simplest case
			// We use the object collision shape, no need to get the collision shape of the compound component  
			if (!obj->getCollisionShape()->isCompound())
			{
				
				m_world->rayTestSingleWithMargin(m_rayFromTrans, m_rayToTrans,
												 obj,
												 shape,
												 obj->getWorldTransform(),
												 m_resultCallback, marginNode);
			}
			// case of a coumpound shape
			else
			{
				t = nodePairContact[i].worldToLocal;
				m_world->rayTestSingleWithMargin(m_rayFromTrans, m_rayToTrans,
												 obj,
												 shape,
												 t,
												 m_resultCallback, marginNode);		
			}
			if (m_resultCallback.hasHit())
			{

				btVector3 correctionBefore = btVector3(0, 0, 0);
				btVector3 correctionAfter = btVector3(0, 0, 0);

				/* Deactivated for now cause it can cause some issues
				
				// Link mouvement to avoid node getting stuck 
				btVector3 correctionBefore = btVector3(0, 0, 0);
				if (indexNode > 0 && !isSphereShape)
				{
					nBefore = &m_nodes[indexNode - 1];
					rlLinkBefore = m_links[indexNode - 1].m_rl;
					btVector3 linkBefore = nBefore->m_x - n->m_x;
					btScalar len = linkBefore.length();
					if (len > rlLinkBefore)
					{
						btVector3 dirBefore = linkBefore / len;
						btVector3 extraSizeLinkBefore = linkBefore - dirBefore * rlLinkBefore;
						btVector3 tangentDirBefore = dirBefore.cross(m_resultCallback.m_hitNormalWorld);
						tangentDirBefore = m_resultCallback.m_hitNormalWorld.cross(tangentDirBefore).normalized();
						//correctionBefore = extraSizeLinkBefore.dot(-m_resultCallback.m_hitNormalWorld) * tangentDirBefore;
					}
				}

				btVector3 correctionAfter = btVector3(0, 0, 0);
				
				 
				// Link mouvement to avoid node getting stuck
				if (indexNode < this->m_nodes.size() - 1 && !isSphereShape)
				{
					nAfter = &m_nodes[indexNode + 1];
					rlLinkAfter = m_links[indexNode].m_rl;
					btVector3 linkAfter = nAfter->m_x - n->m_x;
					btScalar len = linkAfter.length();
					if (len > rlLinkAfter)
					{
						btVector3 dirAfter = linkAfter / len;
						btVector3 extraSizeLinkAfter = linkAfter - dirAfter * rlLinkAfter;
						btVector3 tangentDirAfter = dirAfter.cross(m_resultCallback.m_hitNormalWorld);
						tangentDirAfter = m_resultCallback.m_hitNormalWorld.cross(tangentDirAfter).normalized();
						//correctionAfter = extraSizeLinkAfter.dot(-m_resultCallback.m_hitNormalWorld) * tangentDirAfter;
					}
				}
				*/

				btVector3 contactPoint = m_resultCallback.m_hitPointWorld;

				btVector3 outMouvementPos = m_resultCallback.m_hitNormalWorld * correctionNormal;

				if (obj->getInternalType() == CO_RIGID_BODY)
				{
					btRigidBody* rb = (btRigidBody*)btRigidBody::upcast(obj);
					moveBodyCollision(rb, n, m_resultCallback.m_hitNormalWorld, contactPoint - margin * m_resultCallback.m_hitNormalWorld);
				}
				// Replace node
				btVector3 newPosOut = contactPoint + outMouvementPos + correctionBefore + correctionAfter;
				n->m_x = newPosOut;
				nodePairContact[i].m_Xout = newPosOut ;
			}
		}
	}
}

void btCable::moveBodyCollision(btRigidBody* obj, Node* n, btVector3 normale, btVector3 hitPosition)
{	
	// a = node
	// b = body
	btScalar ima = n->m_im;
	btScalar imb = obj->getInvMass();
	btScalar dt = this->m_sst.sdt;
	btScalar margin = this->m_collisionShape->getMargin();
	if (imb == 0) return;

	btScalar totalMass = ima + imb;
	btTransform wtr = obj->getWorldTransform();
	btMatrix3x3 iwi = obj->getInvInertiaTensorWorld();

	// Contact hardness
	btScalar coef = obj->isStaticOrKinematicObject() ? this->m_cfg.kKHR : this->m_cfg.kCHR;

	// bodyToNodeVector
	btVector3 ra = n->m_x - wtr.getOrigin();
	btVector3 vBody = obj->getVelocityInLocalPoint(ra)*dt;
	btVector3 vNode = n->m_x - n->m_q;
	btVector3 vRelative = vNode - vBody;

	btScalar dn = btDot(vRelative, normale);
	if (dn <= SIMD_EPSILON)
	{
		btScalar offset = (n->m_x - hitPosition).length();
		btMatrix3x3 impulseMat = ImpulseMatrix(dt,ima,imb,iwi,ra);
		auto distPenetration = (-normale * (offset*coef));

		btVector3 impulse = impulseMat *  distPenetration;
		obj->applyImpulse(impulse, ra);
	}
}


void btCable::LRAConstraint()
{
	btScalar distance = 0;

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

void btCable::LRAConstraint(int level, int idxAnchor)
{
	int stablePointIndex = m_anchors[idxAnchor].m_node->index;
	m_anchors[idxAnchor].m_node->m_x = m_anchors[idxAnchor].m_body->getCenterOfMassPosition() + m_anchors[idxAnchor].m_c1;

	// Apply LRA left Side
	for (int j = stablePointIndex - 1; j >= 0; j--)
	{
		Node& n = m_nodes[j];
		btScalar distMax = m_links[stablePointIndex - 1].m_rl;
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
		btScalar distMax = m_links[stablePointIndex].m_rl;
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

void btCable::bendingConstraintDistance()
{
	int size = m_nodes.size();
	float stiffness = this->bendingStiffness;
	float iterationFactor = stiffness * stiffness;
	btScalar angleMax = this->maxAngle;


	for (int i = 1; i < this->m_links.size(); ++i)
	{
		Node* before = m_links[i - 1].m_n[0];  // Node before;
		Node* current = m_links[i].m_n[0];     // Current Node
		Node* after = m_links[i].m_n[1];       // Node After
		btVector3 delta1 = current->m_x - before->m_x;
		btVector3 delta2 = after->m_x - current->m_x;

		if (btFuzzyZero(delta1.length()) || btFuzzyZero(delta2.length())) continue;

		btScalar dot = delta1.normalized().dot(delta2.normalized());

		if (dot < -1.0f) dot = -1.0f;
		if (dot > 1.0f) dot = 1.0f;
		btScalar phi = acos(dot);
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
			btClamp(d2, (btScalar)0.0, rr);
			btClamp(d1, (btScalar)0.0, rr);
			btScalar alpha1 = d2 / rr;
			btScalar alpha2 = d1 / rr;

			btVector3 d = alpha1 * before->m_x + alpha2 * after->m_x - current->m_x;
			btScalar dLen = d.length();

			if (btFuzzyZero(dLen))
			{
				continue;
			}

			btVector3 dNorm = d/dLen;
			btVector3 J1 = alpha1 * dNorm;
			btVector3 J2 = -dNorm;
			btVector3 J3 = alpha2 * dNorm;
			btScalar sum = before->m_im * alpha1 * alpha1 + current->m_im + after->m_im * alpha2 * alpha2;
			if (sum <= DBL_EPSILON)
			{
				continue;
			}
			//btScalar C = dLen;
			btScalar mass = 1.0 / sum;

			btScalar impulse = -stiffness * mass * dLen * iterationFactor;

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
	BT_PROFILE("PSolve_Links");
	for (int i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		Node& a = *l.m_n[0];
		Node& b = *l.m_n[1];
		btVector3 AB = b.m_x - a.m_x;
		btVector3 ABNormalized = AB.normalized();
		btScalar normAB = AB.length();
		btScalar k = m_materials[0]->m_kLST;
		btVector3 errAB = k * (normAB - l.m_rl) * ((1 / normAB) * AB);

		btScalar sumInvMass = a.m_im + b.m_im;
		btVector3 deltap1;
		btVector3 deltap2;
		if (sumInvMass != 0 && a.m_im != 0 && b.m_im != 0)
		{
			deltap1 = a.m_im / sumInvMass * (normAB - l.m_rl) * ABNormalized;
			deltap2 = b.m_im / sumInvMass * (normAB - l.m_rl) * ABNormalized;
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

void btCable::anchorConstraint()
{
	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = m_cfg.kAHR * 1;
	const btScalar dt = m_sst.sdt;
	for (int i = 0, ni = m_anchors.size(); i < ni; ++i)
	{
		const Anchor& a = m_anchors[i];
		const btTransform& t = a.m_body->getWorldTransform();
		Node& n = *a.m_node;
		const btVector3 wa = t * a.m_local;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x) * kAHR;
		const btVector3 impulse = a.m_c0 * vr * a.m_influence;
		n.m_x += impulse * a.m_c2;
		a.m_body->applyImpulse(-impulse, a.m_c1);

		impulses[i] += impulse / dt;
	}
}
#pragma endregion

#pragma region Getter/Setter

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
