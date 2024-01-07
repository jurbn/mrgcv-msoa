#pragma once

#include <nori/object.h>
#include <nori/phase.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief
 * A medium query record, it contains information about the medium interaction.
*/
struct MediumQueryRecord
{
    MediumQueryRecord() : t(0.0f), phaseFunction(nullptr) { }
    MediumQueryRecord(float t, const PhaseFunction *phaseFunction) : t(t), phaseFunction(phaseFunction) { }

    float t;                                ///< The distance to the next medium interactio
    Point3f p;                              ///< The point of interaction in world space
    const PhaseFunction *phaseFunction;     ///< The phase function of the medium interaction
    float pdf;                              ///< The probability density function of the medium interaction
    Color3f sigmaA;                         ///< The absorption coefficient of the medium interaction
    Color3f sigmaT;                         ///< The extinction coefficient of the medium interaction
    Color3f sigmaS;                         ///< The scattering coefficient of the medium interaction
    Color3f Le;                             ///< The emitted radiance of the medium interaction
    Transform worldToMedium;                ///< The transformation from medium to world space 
};

class Medium : public NoriObject {
public:

    /**
     * \brief
     * Medium constructor
     * \param mesh
     * The mesh the medium is attached to
     * \param PhaseFunction
     * The phase function of the medium
    */
    // Medium(const Mesh *mesh, PhaseFunction *phaseFunction) : m_mesh(mesh), m_phaseFunction(phaseFunction) {}

    /**
     * \brief
     * Medium destructor
    */
    virtual ~Medium() {}

    /**
     * \brief
     * Sample a point inside the medium
     * \param sampler
     * The sampler to use for importance sampling
     * \param[out] mqr
     * The medium query record
     * \return
     * The sampled point's properties
    */
    virtual void sample(MediumQueryRecord &mRec, Sampler *sampler) const = 0;

    /**
     * \brief
     *  Sample the distance to the next medium interaction along the given ray
     * \param ray
     * The ray to sample the distance along
     * \param sampler
     * The sampler to use for importance sampling
     * \param[out] t
     * The sampled distance
     * \return
     * True if the distance was sampled, false otherwise
     */
    virtual bool sampleDistance(MediumQueryRecord &mRec, Sampler *sampler) const = 0;

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

    inline const Mesh *getMesh() const { return m_mesh; }

    /**
     * \brief Return the type of object (i.e. Medium/Homogeneous/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EMedium; }

    // Return a human-readable summary
    // virtual std::string toString() const = 0;

protected:
    PhaseFunction *m_phaseFunction;     ///< The phase function of the medium
    Mesh *m_mesh;                       ///< Pointer to the mesh if the medium is attached to a mesh
};

NORI_NAMESPACE_END
