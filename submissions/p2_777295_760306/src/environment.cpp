/*
	This file is part of Nori, a simple educational ray tracer
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
#include <nori/bitmap.h>
#include <nori/warp.h>
#include <filesystem/resolver.h>
#include <fstream>


NORI_NAMESPACE_BEGIN

class EnvironmentEmitter : public Emitter {
public:
	EnvironmentEmitter(const PropertyList& props) {
		m_type = EmitterType::EMITTER_ENVIRONMENT;
		m_environment = 0;

		std::string m_environment_name = props.getString("filename", "null");

		filesystem::path filename =
			getFileResolver()->resolve(m_environment_name);

		std::ifstream is(filename.str());
		if (!is.fail())
		{
			cout << "Loading Environment Map: " << filename.str() << endl;

			m_environment = new Bitmap(filename.str());
			cout << "Loaded " << m_environment_name << " - SIZE [" << m_environment->rows() << ", " << m_environment->cols() << "]" << endl;
		}
		m_radiance = props.getColor("radiance", Color3f(1.));
	}
	~EnvironmentEmitter()
	{
		if (m_environment)
			delete m_environment;
	}

	virtual std::string toString() const {
		return tfm::format(
			"AreaLight[\n"
			"  radiance = %s,\n"
			"  environment = %s,\n"
			"]",
			m_radiance.toString(),
			m_environment_name);
	}

	// We don't assume anything about the visibility of points specified in 'ref' and 'p' in the EmitterQueryRecord.
	virtual Color3f eval(const EmitterQueryRecord& lRec) const {

		// This function call can be done by bsdf sampling routines.
		// Hence the ray was already traced for us - i.e a visibility test was already performed.
		// Hence just check if the associated normal in emitter query record and incoming direction are not backfacing
		if (!m_environment)
			return m_radiance;

		float phi = atan2(lRec.wi[2], lRec.wi[0]);
		float theta = acos(lRec.wi[1]);
		if (phi < 0) phi += 2 * M_PI;

		float x = phi / (2 * M_PI);
		float y = (theta) / M_PI;


		return m_environment->eval(Point2f(x, y))* m_radiance;
	}

	virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample, float optional_u) const
	{
		// if there is no environment map, return black
		if (!m_environment)
			return Color3f(0.f);

		// sample a direction on the unit sphere using the warp function
		Vector3f wi = Warp::squareToUniformSphere(sample);
		// set the direction in the emitter query record
		lRec.dist = (lRec.p - lRec.ref).norm();
		lRec.wi = wi;
		lRec.pdf = pdf(lRec);
		// compute the radiance
		Color3f radiance = eval(lRec);
		return radiance;
	}

	// Returns probability with respect to solid angle given by all the information inside the emitterqueryrecord.
	// Assumes all information about the intersection point is already provided inside.
	// WARNING: Use with care. Malformed EmitterQueryRecords can result in undefined behavior. Plus no visibility is considered.
	virtual float pdf(const EmitterQueryRecord &lRec) const
	{
		// if there is no environment map, return 0
		if (!m_environment)
			return 0.f;
		float pdf = Warp::squareToUniformSpherePdf(lRec.wi);
		return pdf;
	}

	// Get the parent mesh
	void setParent(NoriObject* parent)
	{
	}


protected:
	Color3f m_radiance;
	Bitmap *m_environment;
	std::string m_environment_name;
};

NORI_REGISTER_CLASS(EnvironmentEmitter, "environment")
NORI_NAMESPACE_END
