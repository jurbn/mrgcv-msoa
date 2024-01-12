#include <nori/object.h>
#include <nori/frame.h>
#include <nori/phase.h>

NORI_NAMESPACE_BEGIN
class HenyeyGreenstein : public PhaseFunction {
private:
	float g;
public:
	explicit HenyeyGreenstein(const PropertyList &propList) {
		g = propList.getFloat("g", 0.0f);
	}

	Color3f sample(PhaseFunctionQueryRecord& mRec, const Point2f &sample) const override {
		float cosTheta;
		// g=0
		if (std::abs(g) < 1e-3) cosTheta = 1 - 2 * sample.x();
		else {
			float sqrTerm = (1 - g * g) / (1 - g + 2 * g * sample.x());
			cosTheta = (1 + g * g - sqrTerm * sqrTerm) / (2 * g);
		}
		float theta = acos(cosTheta);
		float phi = 2 * M_PI * sample.y();

		Frame fr(mRec.wi);
		Vector3f localWo(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
		mRec.wo = fr.toWorld(localWo);
		return {1.0f};
	}

	Color3f eval(const PhaseFunctionQueryRecord &mRec) const override {
		float cosTheta = mRec.wi.dot(mRec.wo);
		return (1.0f/(4.0f*M_PI)) * (1.0f - g*g)/pow(1.0f + g*g - 2.0f*g*cosTheta, 1.5);
	}

	float pdf(const PhaseFunctionQueryRecord &mRec) const override {
		float cosTheta = mRec.wi.dot(mRec.wo);
		return (1.0f/(4.0f*M_PI)) * (1.0f - g*g)/pow(1.0f + g*g - 2.0f*g*cosTheta, 1.5);
	}

	std::string toString() const override {
		return tfm::format(
				"HenyeyGreenstein[\n"
				"  g = %f,\n"
				"]",
				g);
	}
};

NORI_REGISTER_CLASS(HenyeyGreenstein, "henyey_greenstein");
NORI_NAMESPACE_END