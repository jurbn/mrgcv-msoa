#include <nori/medium.h>
#include <nori/sampler.h>
#include <nori/density.h>
#include <cmath>

NORI_NAMESPACE_BEGIN

class HeterogeneousMedium : public Medium {
public:
    HeterogeneousMedium(const PropertyList &propList) {
        m_mediumToWorld = propList.getTransform("toWorld", Transform().inverse());
		m_phaseFunction = nullptr;
		m_densityFunction = nullptr;
    }

	/**
	 * \brief Sample a point inside the medium
	*/
	void sample(MediumQueryRecord &mRec, Sampler *sampler) const {
		// printf("Heterogeneous::sample\n");
		// we will sample a point inside the medium and obtain its properties via the density function
        Vector4f sample_color = m_densityFunction->sample(mRec);	// this is a value between 0 and 1, we want a color!
		//given the color we've sampled, we need to transform it to scattering and absorption coefficients
		// the transparence of the medium will be given by the alpha channel of the color
		// if any of the elements of the color is lower than 0, we will consider it as 0
		if ((sample_color.x() < 0.0f) || (sample_color.y() < 0.0f) || (sample_color.z() < 0.0f) || (sample_color.w() < 0.0f))
			sample_color.setZero(4);
		float alpha = sample_color.w();

		if (alpha <= 0.05f)
			alpha = 0.0f;

		mRec.Le = Color3f(alpha)*10.f;
		mRec.sigmaS = Color3f(sample_color.x(), sample_color.y(), sample_color.z()) * alpha;
		// the absorption coefficient will be given by the color
		mRec.sigmaA = Color3f(sample_color.x(), sample_color.y(), sample_color.z()) * alpha;
		// the extinction coefficient will be given by the sum of the scattering and absorption coefficients
		mRec.sigmaT = mRec.sigmaS + mRec.sigmaA;
		// the phase function will be given by the medium's phase function
		mRec.phaseFunction = m_phaseFunction;

	}

	// sampleDistance must select a next medium interaction and return the properties of that interaction (the return must be false if out of the medium)
	bool sampleDistance(MediumQueryRecord &mRec, Sampler *sampler) const {
		// to sample the distance to the next medium interaction, we will use delta tracking
		// given the ray:
		float t = -std::log(1 - sampler->next1D());  // this distance is sampled from an exponential distribution with parameter sigmaT
		// check if the next point of intersection is inside the medium (if not, we're out of the medium)
		sample(mRec, sampler);
		// update the medium query record
		mRec.t = t;
		mRec.pdf = 1.0f;
		return true;
	}

	Color3f evalTransmittance(const Ray3f &ray, Sampler *sampler) const {
		return Color3f(0.0f);
	}

	void addChild(NoriObject *child) {
		if (child->getClassType() == EPhaseFunction) {
			if (m_phaseFunction)
				throw NoriException("Heterogeneous::addChild(): A phase function has already been specified!");
			m_phaseFunction = static_cast<PhaseFunction *>(child);
		} else {
			throw NoriException("Heterogeneous::addChild(): Expected a phase function!");
		}
	}

	void activate() {
		// If no phase function was specified, instantiate an isotropic one
		if (!m_phaseFunction) {
			m_phaseFunction = static_cast<PhaseFunction *>(NoriObjectFactory::createInstance("henyey_greenstein", PropertyList()));
		}
		// If no density function was specified, instantiate a perlin noise one
		if (!m_densityFunction) {
			PropertyList propList;
			propList.setFloat("frequency", 27.0f);
			propList.setInteger("octaves", 3);
			propList.setInteger("persistance", 0.6f);
			propList.setInteger("seed", 0);
			m_densityFunction = static_cast<DensityFunction *>(NoriObjectFactory::createInstance("perlin", propList));
		}
	}

	std::string toString() const {
		return tfm::format(
			"HeterogeneousMedium[\n"
			"  phaseFunction = %s\n"
			"]",
			m_phaseFunction ? (m_phaseFunction->toString()) : std::string("null")
		);
	}

private:
	Transform m_mediumToWorld;	// transform from medium to world space
	DensityFunction *m_densityFunction;	// the density function of the medium
};

NORI_REGISTER_CLASS(HeterogeneousMedium, "heterogeneous");

NORI_NAMESPACE_END