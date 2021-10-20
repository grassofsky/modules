/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2021 Inviwo Foundation
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

#include <inviwo/openmesh/processors/taubinsmootherprocessor.h>
#include <inviwo/openmesh/utils/meshsmoother.h>
#include <inviwo/openmesh/utils/openmeshconverters.h>
#include <inviwo/openmesh/utils/meshotherutils.h>
#include <OpenMesh/Tools/Utils/Timer.hh>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TaubinSmootherProcessor::processorInfo_{
    "org.inviwo.TaubinSmootherProcessor",      // Class identifier
    "Taubin Smoother Processor",                // Display name
    "Mesh Processing",              // Category
    CodeState::Experimental,  // Code state
    Tags::CPU,               // Tags
};
const ProcessorInfo TaubinSmootherProcessor::getProcessorInfo() const { return processorInfo_; }

TaubinSmootherProcessor::TaubinSmootherProcessor()
    : Processor() {
    addPort(inmeshes_);
    addPort(outmeshes_);

    addProperty(iteratorNumber_);
    addProperty(lambda_);
    addProperty(mu_);
    addProperty(enableCalculateVolume_);
}

void TaubinSmootherProcessor::process() {
    //! [Taubin smoothing using openmesh]
    using namespace openmeshutil;
    using TaubinSmoother = TaubinSmootherT<TriMesh>;

    std::vector<std::shared_ptr<Mesh>> meshes;
    for (auto inmesh : inmeshes_)
    {
        auto mesh = fromInviwo(*inmesh, TransformCoordinates::DataToModel);

        auto upsmoother = std::make_unique<TaubinSmoother>(mesh);
        upsmoother->initialize(TaubinSmoother::Tangential_and_Normal, TaubinSmoother::C0);
        upsmoother->set_lambda(lambda_.get());
        upsmoother->set_mu(mu_.get());
        OpenMesh::Utils::Timer timer;
        timer.start();
        upsmoother->smooth(static_cast<unsigned int>(iteratorNumber_.get()));
        timer.stop();

        std::stringstream ss;
        ss << processorInfo_.classIdentifier << " cost time is(ms): " << timer.mseconds();
        util::log(IVW_CONTEXT, ss.str(), LogLevel::Info, LogAudience::User);

        mesh.update_face_normals();
        mesh.update_vertex_normals();

        if (enableCalculateVolume_.get()) {
            std::stringstream ss;
            ss << "Volume Size is: " << calculateTriMeshVolume(mesh) << "\n"
               << "Method: " << processorInfo_.classIdentifier << "\n "
               << "Iter number is: " << iteratorNumber_.get() << "\n"
               << "lambda is: " << lambda_.get() << "\n"
               << "mu is: " << mu_.get();
            util::log(IVW_CONTEXT, ss.str(), LogLevel::Info, LogAudience::User);
        }

        auto newMesh = toInviwo(mesh);
        newMesh->copyMetaDataFrom(*inmesh);
        newMesh->setWorldMatrix(inmesh->getWorldMatrix());
        meshes.push_back(newMesh);
    }

    //! [Taubin smoothing using openmesh]
    if (!meshes.empty())
    {
        outmeshes_.setData(std::make_shared<std::vector<std::shared_ptr<Mesh>>>(meshes));
    }
}

}  // namespace inviwo
