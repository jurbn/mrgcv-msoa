#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct MediumQueryRecord
{
    MediumQueryRecord() : t(0.0f), phaseFunction(nullptr) { }
    MediumQueryRecord(float t, const PhaseFunction *phaseFunction) : t(t), phaseFunction(phaseFunction) { }

    float t;                                ///< The distance to the next medium interaction
    const PhaseFunction *phaseFunction;     ///< The phase function of the medium
};

class Medium : public NoriObject {
public:
    /**
     * \brief
     *  Importance sample the distance to the next medium interaction along the given ray.
     * \param ray
     *  The ray to sample along
     * \param sampler
     *  The sampler to use for importance sampling
     * \param[out] t
     *  The sampled distance to the next medium interaction
     * \param[out] weight
     * The weight of the sampled distance
     * \return
     * True if a distance was sampled, false otherwise
    */
    virtual bool sampleDistance(const Ray3f &ray, Sampler *sampler, float &t) const = 0;

    /**
     * \brief
     *  Evaluate the transmittance along the path segment defined by the ray.
     * \param ray
     *  The ray to evaluate the transmittance along
     * \param sampler
     *  The sampler to use for importance sampling
     * \return
     *  The transmittance along the path segment defined by the ray
    */
    virtual Color3f evalTransmittance(const Ray3f &ray, Sampler *sampler) const = 0;

    // Return the phase function of the medium
    inline const PhaseFunction *getPhaseFunction() const { return m_phaseFunction; }

    // Register a child object with the medium
    virtual void addChild(NoriObject *child) = 0;

    // Initialize the medium
    virtual void activate() = 0;

    /**
     * \brief Return the type of object (i.e. Medium/Homogeneous/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EMedium; }

    // Return a human-readable summary
    // virtual std::string toString() const = 0;

protected:
    PhaseFunction *m_phaseFunction;         ///< The phase function of the medium
    // Mesh *m_mesh;                           ///< The mesh that defines the volume of the medium
};

NORI_NAMESPACE_END
