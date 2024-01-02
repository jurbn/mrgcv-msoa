#include <nori/medium.h>
#include <nori/phase.h>

NORI_NAMESPACE_BEGIN

Medium::Medium() { }

Medium::~Medium() {
    delete m_phaseFunction;
}

void Medium::activate() {
    // If no phase function was specified, instantiate an isotropic one
    if (!m_phaseFunction) {
        m_phaseFunction = static_cast<PhaseFunction *>(NoriObjectFactory::createInstance("isotropic", PropertyList()));
    }
}

void Medium::addChild(NoriObject *child) {
    printf("Medium::addChild()\n");
    if (child->getClassType() == EPhaseFunction) {
        if (m_phaseFunction)
            throw NoriException("Medium::addChild(): A phase function has already been specified!");
        m_phaseFunction = static_cast<PhaseFunction *>(child);
    } else {
        throw NoriException("Medium::addChild(): Expected a phase function!");
    }
}

std::string Medium::toString() const {
    return tfm::format(
        "Medium[\n"
        "  phaseFunction = %s\n"
        "]",
        m_phaseFunction ? (m_phaseFunction->toString()) : std::string("null")
    );
}

NORI_NAMESPACE_END