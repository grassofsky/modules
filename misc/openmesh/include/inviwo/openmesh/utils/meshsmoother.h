/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018-2021 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#ifndef IVW_MESHSMOOTHER_H
#define IVW_MESHSMOOTHER_H

#include <inviwo/openmesh/openmeshmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/util/clock.h>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES_WAS_DEFINED
#define _USE_MATH_DEFINES
#endif

#include <warn/push>
#include <warn/ignore/all>
#include <OpenMesh/Core/IO/MeshIO.hh>  // this needs to be included before TriMesh_ArrayKernelT
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Smoother/LaplaceSmootherT.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>
#include <warn/pop>

#ifdef _USE_MATH_DEFINES_WAS_DEFINED
#undef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES_WAS_DEFINED
#endif

namespace inviwo {

namespace openmeshutil {

/**
 * \class TaubinSmootherT
 * \brief Smooth mesh with algorithm presented in paper
 *        "Taubin G. A signal processing approach to fair surface design"
 */
template <typename OMesh>
class TaubinSmootherT : public OpenMesh::Smoother::LaplaceSmootherT<OMesh> {
private:
    typedef OpenMesh::Smoother::LaplaceSmootherT<OMesh> Base;
    
public:
    explicit TaubinSmootherT(OMesh& mesh) 
        : Base(mesh)
        , lambda_(0.5)
        , mu_(-0.51) {
    }

    void smooth(unsigned int n) override {
        if (Base::continuity() > Base::C0) {
            throw std::invalid_argument("continuity must be C0");
        }

        if (Base::component() == Base::Tangential) {
            throw std::invalid_argument("component must be Normal or Tangential_and_Normal");
        }

        Base::smooth(n);
    }

    /**
     * The lambda is according to the paper
     */
    void set_lambda(double lambda) {
        assert(lambda > 0 && lambda < 1);
        lambda_ = lambda;
    }

    /**
     * The mu is according to the paper
     */
    void set_mu(double mu) {
        assert(mu < 0 && mu > -1.5);
        mu_ = mu;
    }

protected:
    virtual void compute_new_positions_C0() override {
        compute_new_positions_C0_with_lambda(lambda_);

        if (mu_ < -1e-8) {
            typename OMesh::VertexIter v_it, v_end(Base::mesh_.vertices_end());
            for (v_it = Base::mesh_.vertices_begin(); v_it != v_end; ++v_it)
                if (this->is_active(*v_it)) mesh_.set_point(*v_it, this->new_position(*v_it));

            compute_new_positions_C0_with_lambda(mu_);
        }
    }

    virtual void compute_new_positions_C1() override {
        throw std::exception("not implemented");
    }

private:
    void compute_new_positions_C0_with_lambda(double lambda) {
        typename OMesh::VertexIter v_it, v_end(Base::mesh_.vertices_end());
        typename OMesh::ConstVertexOHalfedgeIter voh_it;
        typename OMesh::Normal u, p, zero(0, 0, 0);
        typename OMesh::Scalar w;

        for (v_it = Base::mesh_.vertices_begin(); v_it != v_end; ++v_it) {
            if (this->is_active(*v_it)) {
                // compute umbrella
                u = zero;
                for (voh_it = Base::mesh_.cvoh_iter(*v_it); voh_it.is_valid(); ++voh_it) {
                    w = this->weight(Base::mesh_.edge_handle(*voh_it));
                    u += OpenMesh::vector_cast<typename OMesh::Normal>(
                             Base::mesh_.point(Base::mesh_.to_vertex_handle(*voh_it))) * w;
                }
                u *= this->weight(*v_it);
                u -= OpenMesh::vector_cast<typename OMesh::Normal>(Base::mesh_.point(*v_it));

                // damping
                u *= static_cast<typename OMesh::Scalar>(lambda);

                // store new position
                p = OpenMesh::vector_cast<typename OMesh::Normal>(Base::mesh_.point(*v_it));
                p += u;
                this->set_new_position(*v_it, p);
            }
        }
    }

private:
    double lambda_;
    double mu_;
};

/**
 * \class HCLaplacianSmootherT
 * \brief Smooth mesh with algorithm presented in paper
 *        "Improved Laplacian Smoothing of Noisy Surface Meshes"
 */
template <typename OMesh>
class HCLaplacianSmootherT : public OpenMesh::Smoother::LaplaceSmootherT<OMesh> {
private:
    typedef OpenMesh::Smoother::LaplaceSmootherT<OMesh> Base;

public:
    explicit HCLaplacianSmootherT(OMesh& mesh) : Base(mesh), alpha_(0), beta_(0) {
        Base::mesh_.add_property(first_move_vector_);
    }

    ~HCLaplacianSmootherT() { 
        Base::mesh_.remove_property(first_move_vector_);
    }

    void smooth(unsigned int n) override {
        if (Base::continuity() > Base::C0) {
            throw std::invalid_argument("continuity must be C0");
        }

        if (Base::component() != Base::Tangential_and_Normal) {
            throw std::invalid_argument("component must be Tangential_and_Normal");
        }

        Base::smooth(n);
    }

    /**
     * The alpha is according to the paper, the contribution of original position
     */
    void set_alpha(double alpha) {
        assert(alpha > 0 && alpha < 1);
        alpha_ = alpha;
    }

    /**
     * The beta is according to the paper, the contribution of current position in every iteration
     */
    void set_beta(double beta) {
        assert(beta < 0 && beta > -1.5);
        beta_ = beta;
    }

protected:
    virtual void compute_new_positions_C0() override {

        typename OMesh::VertexIter v_it, v_end(Base::mesh_.vertices_end());
        typename OMesh::ConstVertexOHalfedgeIter voh_it;
        typename OMesh::Normal u, p, q, zero(0, 0, 0);
        typename OMesh::Scalar w;

        // compute first new position by moving forward,
        // and record first move vector
        for (v_it = Base::mesh_.vertices_begin(); v_it != v_end; ++v_it) {
            if (this->is_active(*v_it)) {
                // compute umbrella
                u = zero;
                for (voh_it = Base::mesh_.cvoh_iter(*v_it); voh_it.is_valid(); ++voh_it) {
                    w = this->weight(Base::mesh_.edge_handle(*voh_it));
                    u += OpenMesh::vector_cast<typename OMesh::Normal>(
                             Base::mesh_.point(Base::mesh_.to_vertex_handle(*voh_it))) * w;
                }
                u *= this->weight(*v_it);
                u -= OpenMesh::vector_cast<typename OMesh::Normal>(Base::mesh_.point(*v_it));

                // damping
                u *= static_cast<typename OMesh::Scalar>(0.5);

                // set new position
                q = OpenMesh::vector_cast<typename OMesh::Normal>(Base::mesh_.point(*v_it));
                p = q + u;
                this->set_new_position(*v_it, p);

                // compute first move vector
                set_first_move_vector(*v_it, p - (alpha_ * Base::orig_position(*v_it) + (1 - alpha_) * q));                
            }
        }

        // compute new position by moving back
        for (v_it = Base::mesh_.vertices_begin(); v_it != v_end; ++v_it) {
            if (this->is_active(*v_it)) {
                u = zero;
                for (voh_it = Base::mesh_.cvoh_iter(*v_it); voh_it.is_valid(); ++voh_it) {
                    w = this->weight(Base::mesh_.edge_handle(*voh_it));
                    u += OpenMesh::vector_cast<typename OMesh::Normal>(
                             first_move_vector(Base::mesh_.to_vertex_handle(*voh_it))) * w;
                }
                u *= this->weight(*v_it);
                u = beta_ * first_move_vector(*v_it) + (1 - beta_) * u;

                p = this->new_position(*v_it) - u;
                this->set_new_position(*v_it, p);
            }
        }
    }

    virtual void compute_new_positions_C1() override { throw std::exception("not implemented"); }

private:
    NormalType first_move_vector(VertexHandle _vh) const {
        return Base::mesh_.property(first_move_vector_, _vh);
    }

    void set_first_move_vector(VertexHandle _vh, NormalType normal) {
        Base::mesh_.property(first_move_vector_, _vh) = normal;
    }

private:
    OpenMesh::VPropHandleT<NormalType> first_move_vector_;
    double alpha_;
    double beta_;
};

}  // namespace openmeshutil

}  // namespace inviwo

#endif  // IVW_MESHSMOOTHER_H

