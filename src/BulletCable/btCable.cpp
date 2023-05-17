#include "btCable.h"
#include <BulletSoftBody/btSoftBodyInternals.h>
#include <fstream>
#include <iostream>

#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <Bullet3Common/b3Logging.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#pragma region Forces

void btCable::addSpringForces()
{
	// each link will receive an amout of force depending the spring-mass and the damping
	// 0----/\/\/-----0 Spring-mass : force exerted by spring to restore its relaxed state
	std::ofstream MyFile("addSpringForces.txt");

	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];  // the spring
		Node& na = *l.m_n[0];  // 1st particle
		Node& nb = *l.m_n[1];  // 2nd particle

		btVector3 ab = nb.m_x - na.m_x;       // vector AB
		btScalar lenAB = ab.length();         // length AB
		btVector3 dirAB = ab.normalized();    // direction AB
		btScalar k = m_materials[0]->m_kLST;  // spring stiffness
		btScalar b = m_materials[0]->m_kVST;  // spring bending
		btScalar angle = dirAB.angle(btVector3(0, 1, 0));

		// Hook's law - Axial stiffness
		btVector3 sForce = k * (lenAB - l.m_rl) * dirAB;  // Hook's law on the link's direction
		if (na.m_battach == 0) na.m_f += sForce;
		if (nb.m_battach == 0) nb.m_f -= sForce;

		// Bending stiffness

		MyFile << "Link[" << i << "]: " << std::endl;
		MyFile << "		k(stiffness): " << k << std::endl;
		MyFile << "		Length(AB): " << lenAB << std::endl;
		MyFile << "		Diff length(lij - l0): " << (lenAB - l.m_rl) << std::endl;
		MyFile << "		sForce(AB): " << sForce.length() << std::endl;
	}

	for (int i = 0; i < m_nodes.size(); ++i)
		MyFile << "		Forces n[" << i << "]:" << m_nodes[i].m_f << std::endl
			   << std::endl;

	// Close the file
	MyFile.close();
}

void btCable::addMoorDyn()
{
	// each link will receive an amout of force depending the spring-mass and the damping
	// 0----/\/\/-----0 Spring-mass : force exerted by spring to restore its relaxed state
	// 0----[===|-----0 Damping : reduce the oscillations in the spring
	std::ofstream MyFile("addMoorDyn.txt");

	btScalar I = 3.1415 / 4. * (2 * getCollisionShape()->getMargin()) * (2 * getCollisionShape()->getMargin());
	btScalar E = 3.0e6;
	btScalar C = 3.0e5;

	MyFile << "I: " << I << std::endl;
	MyFile << "E: " << E << std::endl;
	MyFile << "C: " << C << std::endl;
	MyFile << std::endl;

	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l = m_links[i];  // the spring
		Node& na = *l.m_n[0];  // 1st particle
		Node& nb = *l.m_n[1];  // 2nd particle

		btVector3 AB = nb.m_x - na.m_x;  // vector AB
		btScalar lenAB = AB.length();    // length AB
		btVector3 velAB = nb.m_v - na.m_v;
		btVector3 rateStrainAB = AB * velAB / l.m_rl;
		btScalar k = m_materials[0]->m_kLST;
		btScalar d = m_materials[0]->m_kVST;

		btVector3 tForce = k * (lenAB - l.m_rl) * AB.normalized();
		if (na.m_battach == 0) na.m_f += tForce;
		if (nb.m_battach == 0) nb.m_f -= tForce;


		MyFile << "Link[" << i << "]: " << std::endl;
		MyFile << "		K(stiffness): " << E * I << std::endl;
		MyFile << "		C(damping): " << C * I << std::endl;
		MyFile << "		Length(AB): " << lenAB << std::endl;
		MyFile << "		Ratio (AB): " << (AB / l.m_rl).length() << std::endl;
		MyFile << "		Diff. length: " << (lenAB - l.m_rl) << std::endl;
		MyFile << "		Vel. AB: " << velAB.length() << std::endl;
		MyFile << "		rate of strain: " << rateStrainAB.length() << std::endl;
		MyFile << "		sForce(AB): " << tForce.length() << std::endl;
	}

	MyFile << std::endl;
	for (int i = 0; i < m_nodes.size(); ++i)
		MyFile << "Forces n[" << i << "]:" << m_nodes[i].m_f.length() << std::endl;
	MyFile << std::endl;
}

void btCable::addForces()
{
	std::ofstream MyFile("addForces.txt");

	for (int i = 0; i < m_links.size(); ++i)
	{
		Link& l  = m_links[i]; // the spring
		Node& na = *l.m_n[0];  // 1st particle
		Node& nb = *l.m_n[1];  // 2nd particle

		btVector3 distAB  = nb.m_x - na.m_x;     // distance AB
		btVector3 velAB   = nb.m_v - na.m_v;     // velocity AB
		btScalar lenAB    = distAB.length();     // length AB
		btVector3 dirAB   = distAB.normalized(); // direction AB

		btScalar mr   = 1 / (na.m_im + nb.m_im);	  // mass reduced
		btScalar k    = m_materials[0]->m_kLST;		  // spring stiffness
		btScalar kMax = mr / (m_sst.sdt * m_sst.sdt); // spring stiffness max
		btScalar d    = m_materials[0]->m_kVST;		  // spring damping
		btScalar dMax = mr / m_sst.sdt;			      // spring damping max

		// Forces
		btVector3 sForce = k * kMax * (lenAB - l.m_rl) * dirAB;  // spring force
		if (na.m_battach == 0) na.m_f += sForce;
		if (nb.m_battach == 0) nb.m_f -= sForce;

		btScalar velA = dirAB.dot(na.m_v);
		btScalar velB = dirAB.dot(nb.m_v);
		btVector3 dForce = d * dMax * dirAB.dot(velAB) * dirAB;  // damping force
		// if (na.m_battach == 0) na.m_f += dForce;
		// if (nb.m_battach == 0) nb.m_f -= dForce;

		btVector3 gForce = 0.5 * m_worldInfo->m_gravity * mr; // gravity force
		if (na.m_battach == 0) na.m_f += gForce;
		if (nb.m_battach == 0) nb.m_f += gForce;

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
}


struct btBridgedManifoldResult : public btManifoldResult
{
	btCollisionWorld::ContactResultCallback& m_resultCallback;

	btBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap, const btCollisionObjectWrapper* obj1Wrap, btCollisionWorld::ContactResultCallback& resultCallback)
		: btManifoldResult(obj0Wrap, obj1Wrap),
		  m_resultCallback(resultCallback)
	{
		
	}

	virtual void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth)
	{
		ofstream myfilee;
		myfilee.open("AA.txt");
		myfilee << "Here";
		myfilee.close();

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
/*
void btCable::solveConstraints() {

	int i, ni;

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
	// Solve positions		
	if (m_cfg.piterations > 0)
	{
		for (int isolve = 0; isolve < m_cfg.piterations; ++isolve)
		{
			for (i = 0, ni = m_anchors.size(); i < ni; ++i)
				impulses[i] = btVector3(0, 0, 0);

			const btScalar ti = isolve / (btScalar)m_cfg.piterations;
			for (int iseq = 0; iseq < 2; ++iseq)
			{
				btCable::getSolver(m_cfg.m_psequence[iseq])(this, 1, ti);
			}
		}

		const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
		for (i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			n.m_v = (n.m_x - n.m_q) * vc;
			n.m_f = btVector3(0, 0, 0);
		}
	}
}
*/

//
void btCable::solveConstraints()
{
	/* Apply clusters		*/
	applyClusters(false);
	/* Prepare links		*/

	int i, ni;

	for (i = 0, ni = m_links.size(); i < ni; ++i)
	{
		Link& l = m_links[i];
		l.m_c3 = l.m_n[1]->m_q - l.m_n[0]->m_q;
		l.m_c2 = 1 / (l.m_c3.length2() * l.m_c0);
	}
	/* Prepare anchors		*/
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
	/* Solve velocities		*/
	if (m_cfg.viterations > 0)
	{
		/* Solve			*/
		for (int isolve = 0; isolve < m_cfg.viterations; ++isolve)
		{
			for (int iseq = 0; iseq < m_cfg.m_vsequence.size(); ++iseq)
			{
				btSoftBody::getSolver(m_cfg.m_vsequence[iseq])(this, 1);
			}
		}
		/* Update			*/
		for (i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			n.m_x = n.m_q + n.m_v * m_sst.sdt;
		}
	}
	/* Solve positions		*/
	if (m_cfg.piterations > 0)
	{
		for (int isolve = 0; isolve < m_cfg.piterations; ++isolve)
		{
			const btScalar ti = isolve / (btScalar)m_cfg.piterations;
			for (int iseq = 0; iseq < m_cfg.m_psequence.size(); ++iseq)
			{
				getSolver(m_cfg.m_psequence[iseq])(this, 1, ti);
			}
		}
		const btScalar vc = m_sst.isdt * (1 - m_cfg.kDP);
		for (i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			n.m_v = (n.m_x - n.m_q) * vc;
			n.m_f = btVector3(0, 0, 0);
		}
	}
	/* Solve drift			*/
	if (m_cfg.diterations > 0)
	{
		const btScalar vcf = m_cfg.kVCF * m_sst.isdt;
		for (i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			n.m_q = n.m_x;
		}
		for (int idrift = 0; idrift < m_cfg.diterations; ++idrift)
		{
			for (int iseq = 0; iseq < m_cfg.m_dsequence.size(); ++iseq)
			{
				getSolver(m_cfg.m_dsequence[iseq])(this, 1, 0);
			}
		}
		for (int i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			n.m_v += (n.m_x - n.m_q) * vcf;
		}
	}
	/* Apply clusters		*/
	dampClusters();
	applyClusters(true);
}

#pragma endregion


void btCable::predictMotion(btScalar dt)
{
	int i, ni;

	/* Update                */
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

	/* Prepare                */
	m_sst.sdt = dt * m_cfg.timescale;
	m_sst.isdt = 1 / m_sst.sdt;
	m_sst.velmrg = m_sst.sdt * 3;
	m_sst.radmrg = getCollisionShape()->getMargin();
	m_sst.updmrg = m_sst.radmrg * (btScalar)0.25;
	/* Forces                */
	addVelocity(m_worldInfo->m_gravity * m_sst.sdt);
	applyForces();
	/* Integrate            */
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
	/* Clusters                */
	updateClusters();
	/* Bounds                */
	updateBounds();
	/* Nodes                */
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
	/* Faces                */
	if (!m_fdbvt.empty())
	{
		for (int i = 0; i < m_faces.size(); ++i)
		{
			Face& f = m_faces[i];
			const btVector3 v = (f.m_n[0]->m_v +
								 f.m_n[1]->m_v +
								 f.m_n[2]->m_v) /
								3;
			vol = VolumeOf(f, m_sst.radmrg);
			m_fdbvt.update(f.m_leaf,
						   vol,
						   v * m_sst.velmrg,
						   m_sst.updmrg);
		}
	}
	/* Pose                    */
	updatePose();
	/* Match                */
	if (m_pose.m_bframe && (m_cfg.kMT > 0))
	{
		const btMatrix3x3 posetrs = m_pose.m_rot;
		for (int i = 0, ni = m_nodes.size(); i < ni; ++i)
		{
			Node& n = m_nodes[i];
			if (n.m_im > 0)
			{
				const btVector3 x = posetrs * m_pose.m_pos[i] + m_pose.m_com;
				n.m_x = Lerp(n.m_x, x, m_cfg.kMT);
			}
		}
	}
	/* Clear contacts        */
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	/* Optimize dbvt's        */
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
	m_cdbvt.optimizeIncremental(1);
}

/*
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
	addForces(); // spring-mass

	// Integrate (old: Euler semi-Implicit) 
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

	// Integrate (new)
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		if (n.m_battach == 0)
		{
			n.m_q = n.m_x;

			btVector3 acc = n.m_f * n.m_im;
			n.m_x += n.m_v * m_sst.sdt + 0.5 * acc * m_sst.sdt * m_sst.sdt;
			n.m_v += acc * m_sst.sdt;
			n.m_v *= (1 - m_cfg.kDP);
	
			n.m_f = btVector3(0, 0, 0);
		}
	}
	

	// Integrate (new: Implicit)
	for (i = 0, ni = m_nodes.size(); i < ni; ++i)
	{
		Node& n = m_nodes[i];
		if (n.m_battach == 0)
		{
			n.m_v +=  

			n.m_f = btVector3(0, 0, 0);
		}
	}
	

	// Bounds               
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

	// Clear contacts       
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	// Optimize dbvt's
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
	m_cdbvt.optimizeIncremental(1);
}
*/
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

/*
void btCable::PSolve_Anchors(btSoftBody* psb, btScalar kst, btScalar ti)
{
	btCable* cable = (btCable*)psb;

	BT_PROFILE("PSolve_Anchors");
	const btScalar kAHR = psb->m_cfg.kAHR * kst;
	const btScalar dt = psb->m_sst.sdt;
	for (int i = 0, ni = psb->m_anchors.size(); i < ni; ++i)
	{
		Anchor& a = psb->m_anchors[i];
		const btTransform& t = a.m_body->getWorldTransform();
		Node& n = *a.m_node;
		const btVector3 wa = t * a.m_local;                                   // world pos
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;  // speed rel.
		const btVector3 vb = n.m_x - n.m_q;                                   // speed node
		const btVector3 vr = (va - vb) + (wa - n.m_x) * kAHR;
		btVector3 impulse = a.m_c0 * vr * a.m_influence;
		cable->impulses[i] += -impulse / dt;
		a.m_body->applyImpulse(-impulse, a.m_c1);
		n.m_x += impulse * a.m_c2;
	}
}*/

//
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
	for (int i = 0; i < psb->m_links.size(); ++i)
	{
		Link& l = psb->m_links[i];
		if (l.m_c0 > 0 && l.m_c0)
		{
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
						if (a.m_n != btVector3(0, 0, 0))
						{
							// BlackHole method (use the node's normal)
							btVector3 dirBlackHole = a.m_n - a.m_x;
							btScalar forceBlackHole = 1 / dirBlackHole.length();
							a.m_x -= dirBlackHole * forceBlackHole * k;
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
	
	
	for (int i = 0, ni = psb->m_rcontacts.size(); i < ni; ++i)
	{

		const RContact& c = psb->m_rcontacts[i];
		const sCti& cti = c.m_cti;
		if (cti.m_colObj->hasContactResponse() && c.m_node->m_battach == 0) // Collision with anchor disable
		{
			btVector3 va(0, 0, 0);
			btRigidBody* rigidCol = 0;
			btMultiBodyLinkCollider* multibodyLinkCol = 0;
			btScalar* deltaV = NULL;

			if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
				va = rigidCol ? rigidCol->getVelocityInLocalPoint(c.m_c1) * dt : btVector3(0, 0, 0);
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
					va = cti.m_normal * vel * dt;
				}
			}

			const btVector3 vb = c.m_node->m_x - c.m_node->m_q;
			const btVector3 vr = vb - va;
			const btScalar dn = btDot(vr, cti.m_normal);
			if (dn <= SIMD_EPSILON)
			{
				const btScalar dp = btMin((btDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg);
				const btVector3 fv = vr - (cti.m_normal * dn);
				// c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
				const btVector3 impulse = c.m_c0 * ((vr - (fv * c.m_c3) + (cti.m_normal * (dp * c.m_c4))) * kst);
				c.m_node->m_x -= impulse * c.m_c2;

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
	impulses = new btVector3[2]{btVector3(0, 0, 0)};
	tempManiforld = new btPersistentManifold();
	void* mem = btAlignedAlloc(sizeof(btGjkEpaPenetrationDepthSolver), 16);
	m_pdSolver = new (mem) btGjkEpaPenetrationDepthSolver;
	for (int i = 0; i < this->m_nodes.size(); i++)
	{
		m_nodes[i].m_battach = 0;
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

void btCable::setRestLenghtLink(int index, btScalar distance)
{
	Link& l = m_links[index];
	l.m_rl = distance;
	l.m_c1 = l.m_rl * l.m_rl;
}

btScalar btCable::getRestLenghtLink(int index)
{
	Link& l = m_links[index];
	return l.m_rl;
}

void btCable::swapNodes(int index0, int index1)
{
	m_nodes.swap(index0, index1);
}

void btCable::swapAnchors(int index0, int index1)
{
	m_anchors.swap(index0, index1);
}

btScalar btCable::getLength()
{
	btScalar length = 0;
	for (int i = 0; i < m_links.size(); ++i)
		length += m_links[i].m_rl;
	return length;
}

btVector3* btCable::getImpulses()
{
	return impulses;
}

btCollisionShape* btCable::getCollisionShapeNode() const{
	return collisionShapeNode;
}


void btCable::setCollisionShapeNode(btCollisionShape* nodeShape) {
	this->collisionShapeNode = nodeShape;
}


bool btCable::checkContact(const btCollisionObjectWrapper* colObjWrap,
							  const btVector3& x,
							  btScalar margin,
							  btSoftBody::sCti& cti) const{
	
	btVector3 nrm;
	const btCollisionShape* shp = colObjWrap->getCollisionShape();
	const btCollisionObject* obj = colObjWrap->getCollisionObject();
	btCollisionObject tempo = btCollisionObject(*obj);

	const btTransform& wtr = colObjWrap->getWorldTransform();
	
	struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback
	{
		bool m_connected;
		btScalar m_margin;
		btScalar m_dist=10000000.0;
		btVector3 norm;
		MyContactResultCallback(float margin) : m_connected(false), m_margin(margin), norm(btVector3(0,0,0))
		{
		}
		virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
		{
			if (cp.getDistance() <= m_margin)
			{
				m_connected = true;
				if (cp.getDistance() < m_dist)
				{
					m_dist = cp.getDistance() - m_margin;
					norm = cp.m_normalWorldOnB;
				}	 
			}
			return 1.f;
		} 
	};
 
	MyContactResultCallback result(margin);

	/*
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
	 
	btCollisionObject nodeShape;
	btTransform temp = btTransform();
	temp.setIdentity();
	temp.setOrigin(x);
	nodeShape.setWorldTransform(temp);
	nodeShape.setCollisionShape(new btSphereShape(margin));
	
	checkCollide(&nodeShape, &tempo, result);
	
	//world->contactPairTest(&nodeShape, &tempo, result);
	
	if (result.m_connected)
	{		
		cti.m_colObj = colObjWrap->getCollisionObject();
		cti.m_normal = result.norm;
		cti.m_offset = -btDot(cti.m_normal, x - cti.m_normal * result.m_dist);		
		return (true);
	}
	
	return (false);
}

void btCable::setWorldRef(btCollisionWorld* colWorld)
{
	this->world = colWorld;
}

void btCable::checkCollide(btCollisionObject* colObjA, btCollisionObject* colObjB, btCollisionWorld::ContactResultCallback& resultCallback) const 
{	
	btCollisionObjectWrapper obA(0, colObjA->getCollisionShape(), colObjA, colObjA->getWorldTransform(), -1, -1);
	btCollisionObjectWrapper obB(0, colObjB->getCollisionShape(), colObjB, colObjB->getWorldTransform(), -1, -1);
	
	//btCollisionAlgorithm* algorithm = m_worldInfo->m_dispatcher->findAlgorithm(&obA, &obB, 0, BT_CLOSEST_POINT_ALGORITHMS);
	
	/*
	btCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher1 = world->getDispatcher();

	void* membis = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btConvexConvexAlgorithm));
	//btCollisionAlgorithmCreateFunc *m_convexConvexCreateFunc = new (mem) btConvexConvexAlgorithm::CreateFunc(m_pdSolver);
	//default CreationFunctions, filling the m_doubleDispatch table
		
	btCollisionAlgorithm* Tempalgorithm = new (membis) btConvexConvexAlgorithm(tempManiforld, ci, &obA, &obB, m_pdSolver, 3, 0);
	*/
	btCollisionAlgorithm* Tempalgorithm = m_worldInfo->m_dispatcher->findAlgorithm(&obA, &obB, tempManiforld, BT_CLOSEST_POINT_ALGORITHMS);
	
	if (Tempalgorithm)
	{
		// Pass here
		btBridgedManifoldResult contactPointResult(&obA, &obB, resultCallback);
		contactPointResult.m_closestPointDistanceThreshold = resultCallback.m_closestDistanceThreshold;
		//discrete collision detection query
		Tempalgorithm->processCollision(&obA, &obB, world->getDispatchInfo(), &contactPointResult);
		Tempalgorithm->~btCollisionAlgorithm();
		m_worldInfo->m_dispatcher->freeCollisionAlgorithm(Tempalgorithm);
	}
	tempManiforld->clearManifold();
	
}



