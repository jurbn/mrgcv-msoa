#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN
class DepthIntegrator : public Integrator {
    public :
    DepthIntegrator(const PropertyList& props) {
    /* No parameters this time */
    }

    Color3f Li(const Scene* scene , Sampler* sampler , const Ray3f& ray) const {
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))     // if the ray does not intersect, 
            return scene->getBackground(ray);   // return the color of the background
        EmitterQueryRecord emitterRecord(its.p);
        // Calculate the distance from the camera to the intersection point
        float dist = (its.p - ray.o).norm();
        // Transform that distance into a color of value 1/dist
        return Color3f(1.f/dist);
    }

    std::string toString( ) const {
        return "Depth Integrator []" ;
    }
    
};
NORI_REGISTER_CLASS(DepthIntegrator , "depth" ) ;
NORI_NAMESPACE_END