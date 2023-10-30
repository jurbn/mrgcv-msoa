/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    v1 - Dec 2020
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

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() {}

Mesh::~Mesh()
{
    m_pdf.clear();
    delete m_bsdf;
    delete m_emitter;
}

void Mesh::activate()
{
    if (!m_bsdf)
    {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }

    m_pdf.reserve(m_F.cols()); // reserve memory for the number of faces in the mesh
    m_pdf.normalize();         // normalize the pdf, getNormalization() from dpdf.h
}

float Mesh::surfaceArea(n_UINT index) const
{
    // get the faces of triangle with index
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    // get the vertices of the triangle
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    // compute the area of the triangle
    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(n_UINT index, const Ray3f &ray, float &u, float &v, float &t) const
{
    n_UINT i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

BoundingBox3f Mesh::getBoundingBox(n_UINT index) const
{
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(n_UINT index) const
{
    return (1.0f / 3.0f) *
           (m_V.col(m_F(0, index)) +
            m_V.col(m_F(1, index)) +
            m_V.col(m_F(2, index)));
}

/**
 * \brief Uniformly sample a position on the mesh with
 * respect to surface area. Returns both position and normal
 */
void Mesh::samplePosition(const Point2f &sample, Point3f &p, Normal3f &n, Point2f &uv) const
{
    // samples a point in a triangle of the mesh

    // Sample a triangle of the mesh at random, with probability proportional to the triangle area

    Point2f randomSample = sample;
    size_t triangleIndex = m_pdf.sampleReuse(randomSample.x());

    // Get the vertices of the triangle
    // m_V contains the vertex positions
    // m_F stores the faces of the mesh
    /* we are accessing the first (and only) triangle in the mesh. m_F(0, 0)
    refers to the first column (vertex) of the first row (triangle).*/

    n_UINT i0 = m_F(0, triangleIndex), i1 = m_F(1, triangleIndex), i2 = m_F(2, triangleIndex); // vertex indices (within the faces)
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);                        // vertex positions

    // Sample barycentric coordinates that define the position of the point relative to the triangle's vertices
    const Point2f barycentric = Warp::squareToUniformTriangle(sample);
    float u = barycentric.x(); // barycentric.x();
    float v = barycentric.y();
    float w = 1.0f - u - v; // the sum of the barycentric coordinates must be 1

    // Interpolate the position
    // how much do we move along the edges of the triangle
    p = p0 * u + p1 * v + p2 * w;

    // Compute the normal of the triangle
    /* The cross product of two vectors yields a vector that is orthogonal (perpendicular) to both input vectors.
    In the context of a triangle, this will be the normal to the triangle's surface.*/
    const Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    // if the mesh has normals, use them
    const Point3f n0 = m_N.col(i0), n1 = m_N.col(i1), n2 = m_N.col(i2); // vertex normals
    n = (n0 * u + n1 * v + n2 * w);                                     // .normalized();                        // interpolate the normals

    // if the mesh does not have normals, compute them
    n = (edge1.cross(edge2)).normalized();

    // Interpolate the UV coordinates if they exist
    if (m_UV.cols() > 0)
    {
        const Point2f uv0 = m_UV.col(i0), uv1 = m_UV.col(i1), uv2 = m_UV.col(i2); // UV coordinates
        uv = uv0 * u + uv1 * v + uv2 * w;                                         // interpolate the UV coordinates
    }
    else
    {
        uv = Point2f::Zero();
    }
}

/// Return the surface area of the given triangle
float Mesh::pdf(const Point3f &p) const
{
    // Initialize the area of the triangle
    float area = 0.0f;
    // Iterate over all the triangles in the mesh (faces then vertices)
    for (uint32_t i = 0; i < m_F.cols(); i++)
    {
        area + surfaceArea(i); // add the area of the current triangle to the total area
    }

    // Return the area of the triangle
    // uniform distribution -> 1/area
    return 1.0f / area;
}

void Mesh::addChild(NoriObject *obj, const std::string &name)
{
    switch (obj->getClassType())
    {
    case EBSDF:
        if (m_bsdf)
            throw NoriException(
                "Mesh: tried to register multiple BSDF instances!");
        m_bsdf = static_cast<BSDF *>(obj);
        break;

    case EEmitter:
    {
        Emitter *emitter = static_cast<Emitter *>(obj);
        if (m_emitter)
            throw NoriException(
                "Mesh: tried to register multiple Emitter instances!");
        m_emitter = emitter;
    }
    break;

    default:
        throw NoriException("Mesh::addChild(<%s>) is not supported!",
                            classTypeName(obj->getClassType()));
    }
}

std::string Mesh::toString() const
{
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(), // face count
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null"));
}

std::string Intersection::toString() const
{
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null"));
}

NORI_NAMESPACE_END
