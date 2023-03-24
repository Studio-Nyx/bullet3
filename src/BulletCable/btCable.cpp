#include "btCable.h"
#include <BulletSoftBody/btSoftBodyInternals.h>

void btCable::solveConstraints() {
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

btSoftBody::psolver_t btCable::getSolver(ePSolver::_ solver)
{
	switch (solver)
	{
		case ePSolver::Anchors:
			return (&btCable::PSolve_Anchors);
		case ePSolver::Linear:
			return (&btSoftBody::PSolve_Links);
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

void btCable::PSolve_Anchors(btSoftBody* psb, btScalar kst, btScalar ti)
{
	btCable* cable = (btCable*) psb;
	cable->impulses = new btVector3[psb->m_anchors.size()]{btVector3(0, 0, 0)};

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
		cable->impulses[i] = impulse / dt;
		n.m_x += impulse * a.m_c2;
		a.m_body->applyImpulse(-impulse, a.m_c1);
	}
}

btCable::btCable(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m) 
{
	impulses = new btVector3[2]{btVector3(0, 0, 0)};
}

void btCable::removeLink(int index) {
	m_links.removeAtIndex(index);
}

void btCable::removeNode(int index) {
	m_nodes.removeAtIndex(index);
}

void btCable::removeAnchor(int index)
{
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

