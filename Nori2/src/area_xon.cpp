/*
	This file is part of Nori, a simple educational ray tracer

	Copyright (c) 2015 by Wenzel Jakob

	v1 - Dec 01 2020
	Copyright (c) 2020 by Adrian Jarabo

	Nori is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License Version 3
	as published by the Free Software Foundation.

	Nori is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/mesh.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter
{
public:
	AreaEmitter(const PropertyList &props)
	{
		m_type = EmitterType::EMITTER_AREA;
		m_radiance = new ConstantSpectrumTexture(props.getColor("radiance", Color3f(1.f)));
		m_scale = props.getFloat("scale", 1.);
	}

	virtual std::string toString() const
	{
		return tfm::format(
			"AreaLight[\n"
			"  radiance = %s,\n"
			"  scale = %f,\n"
			"]",
			m_radiance->toString(), m_scale);
	}

	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
	virtual Color3f eval(const EmitterQueryRecord &lRec) const
	{
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		// This function call can be done by bsdf sampling routines.
		// Hence the ray was already traced for us - i.e a visibility test was already performed.
		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing

		// Ensure that the outgoing direction is above the emitting hemisphere of the triangle
		if (lRec.wi.dot(-lRec.n) < 0.0f)
		{
			// Access the radiance texture to compute the radiance
			Color3f radiance = m_radiance->eval(lRec.uv);
			// Scale the radiance
			return radiance * m_scale; // radiance * lRec.pdf
		}
		else
		{
			return Color3f(0.f); // Return black if the direction is below the emitting hemisphere
		}
	}

	virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample, float optional_u) const
	{
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		if (lRec.n.dot(-lRec.wi) < 0.0f)
		{
			return Color3f(0.f); // Return black if the direction is below the emitting hemisphere
		}
		// Sample the point on the mesh
		m_mesh->samplePosition(sample, lRec.p, lRec.n, lRec.uv);
		// Compute the direction from the reference point to the sampled point
		lRec.dist = (lRec.p - lRec.ref).norm();
		lRec.wi = (lRec.p - lRec.ref) / lRec.dist;
		// Compute the probability of sampling the point
		lRec.pdf = pdf(lRec); // pdf(lRec);
		if (lRec.pdf < 1e-3f)
			return Color3f(0.f); // Return black if the direction is below the emitting hemisphere
		// Access the radiance texture to compute the radiance
		Color3f radiance = m_radiance->eval(lRec.uv) / pow(lRec.dist, 1);
		return radiance;
	}

	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior.
	//			Plus no visibility is considered.
	virtual float pdf(const EmitterQueryRecord &lRec) const
	{
		if (!m_mesh)
			throw NoriException("There is no shape attached to this Area light!");

		if (lRec.n.dot(-lRec.wi) < 0.0f)
		{
			return 0.f; // Return black if the direction is below the emitting hemisphere
		}
		// Compute the probability of sampling the point
		float pdf = m_mesh->pdf(lRec.p);
		float dist2 = static_cast<float>(pow(lRec.dist, 2));
		float cosFactor = lRec.n.dot(-lRec.wi);
	}

	// Get the parent mesh
	void setParent(NoriObject *parent)
	{
		auto type = parent->getClassType();
		if (type == EMesh)
			m_mesh = static_cast<Mesh *>(parent);
	}

	// Set children
	void addChild(NoriObject *obj, const std::string &name = "none")
	{
		switch (obj->getClassType())
		{
		case ETexture:
			if (name == "radiance")
			{
				delete m_radiance;
				m_radiance = static_cast<Texture *>(obj);
			}
			else
				throw NoriException("AreaEmitter::addChild(<%s>,%s) is not supported!",
									classTypeName(obj->getClassType()), name);
			break;

		default:
			throw NoriException("AreaEmitter::addChild(<%s>) is not supported!",
								classTypeName(obj->getClassType()));
		}
	}

protected:
	Texture *m_radiance;
	float m_scale;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END
