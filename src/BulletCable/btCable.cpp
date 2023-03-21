#include "btCable.h"
#include <BulletSoftBody/btSoftBodyInternals.h>

btCable::btCable(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m) : btSoftBody(worldInfo, node_count, x, m) {}

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

btVector3* btCable::getImpulseAnchors(int nb)
{
	btVector3* impulses = new btVector3[nb]{btVector3(0, 0, 0)};
	const btScalar dt = m_sst.sdt;
	for (int i = 0; i < nb; ++i)
	{
		const Anchor& a = m_anchors[i];
		const btTransform& t = a.m_body->getWorldTransform();
		Node& n = *a.m_node;
		const btVector3 wa = t * a.m_local;
		const btVector3 va = a.m_body->getVelocityInLocalPoint(a.m_c1) * dt;
		const btVector3 vb = n.m_x - n.m_q;
		const btVector3 vr = (va - vb) + (wa - n.m_x) * m_cfg.kAHR;
		impulses[i] = a.m_c0* vr* a.m_influence;
	}
	return impulses;
}


