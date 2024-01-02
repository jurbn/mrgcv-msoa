#include <nori/medium.h>
#include <nori/phase.h>

NORI_NAMESPACE_BEGIN

class HomogeneousMedium : public Medium {
public:
    HomogeneousMedium(const PropertyList &propList) {
        m_sigmaS = propList.getColor("sigmaS", Color3f(0.0f));
        m_sigmaA = propList.getColor("sigmaA", Color3f(0.0f));
        m_mediumToWorld = propList.getTransform("toWorld", Transform().inverse());
    }

    bool sampleDistance(const Ray3f &ray, Sampler *sampler, float &t, Color3f &weight) const {
		throw NoriException("HomogeneousMedium::sampleDistance(): not implemented!");
	}

	Color3f evalTransmittance(const Ray3f &ray, Sampler *sampler) const {
		throw NoriException("HomogeneousMedium::evalTransmittance(): not implemented!");
	}

	void addChild(NoriObject *child) {
		printf("Homogeneous::addChild()\n");
		if (child->getClassType() == EPhaseFunction) {
			if (m_phaseFunction)
				throw NoriException("Homogeneous::addChild(): A phase function has already been specified!");
			m_phaseFunction = static_cast<PhaseFunction *>(child);
		} else {
			throw NoriException("Homogeneous::addChild(): Expected a phase function!");
		}
	}

	void activate() {
		// If no phase function was specified, instantiate an isotropic one
		if (!m_phaseFunction) {
			m_phaseFunction = static_cast<PhaseFunction *>(NoriObjectFactory::createInstance("isotropic", PropertyList()));
		}
	}

	std::string toString() const {
		return tfm::format(
			"HomogeneousMedium[\n"
			"  phaseFunction = %s\n"
			"  sigmaS = %1,\n"
			"  sigmaA = %2,\n"
			"]",
			m_phaseFunction ? (m_phaseFunction->toString()) : std::string("null"),
			m_sigmaS.toString(),
			m_sigmaA.toString()
		);
	}

private:
	Color3f m_sigmaS;
	Color3f m_sigmaA;
	Transform m_mediumToWorld;
};

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous");

NORI_NAMESPACE_END