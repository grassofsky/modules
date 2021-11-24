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

#include <inviwo/gslaboratory/processors/singlesurfaceextractionprocessor.h>
#include <inviwo/core/util/rendercontext.h>
#include <modules/base/algorithm/volume/marchingcubes.h>
#include <modules/base/algorithm/volume/marchingcubesopt.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SingleSurfaceExtractionProcessor::processorInfo_{
    "org.inviwo.SingleSurfaceExtractionProcessor",      // Class identifier
    "Single Surface Extraction Processor",                // Display name
    "Mesh Creation",              // Category
    CodeState::Experimental,  // Code state
    Tags::CPU,               // Tags
};
const ProcessorInfo SingleSurfaceExtractionProcessor::getProcessorInfo() const { return processorInfo_; }

SingleSurfaceExtractionProcessor::SingleSurfaceExtractionProcessor()
    : Processor()
    , inport_("volume")
    , outport_("mesh")
    , isoValue_("iso", "ISO Value", 0.5f, 0.0f, 1.0f, 0.01f)
    , invertIso_("invert", "Invert ISO", false)
    , encloseSurface_("enclose", "Enclose Surface", true)
    , meshColor_("meshColor", "Mesh Color", vec4(1.0f)) {

    addPort(inport_);
    addPort(outport_);

    addProperty(isoValue_);
    addProperty(invertIso_);
    addProperty(encloseSurface_);
    addProperty(meshColor_);

    inport_.onChange([this]() { 
        if (inport_.hasData()) {
            isoValue_.setMinValue(static_cast<float>(inport_.getData()->dataMap_.dataRange.x));
            isoValue_.setMaxValue(static_cast<float>(inport_.getData()->dataMap_.dataRange.y));
        }
    });
}

void SingleSurfaceExtractionProcessor::process() {
    const auto computeSurface = [this](vec4 color, std::shared_ptr<const Volume> vol) -> std::shared_ptr<Mesh> {
        auto iso = isoValue_.get();
        auto invert = invertIso_.get();
        auto enclose = encloseSurface_.get();
        RenderContext::getPtr()->activateLocalRenderContext();

        return util::marchingCubesOpt(vol, iso, color, invert, enclose);
    };

    const auto changeColor = [](vec4 color, std::shared_ptr<const Mesh> oldmesh) -> std::shared_ptr<Mesh> {
        RenderContext::getPtr()->activateLocalRenderContext();

        auto mesh = std::make_shared<Mesh>(oldmesh->getDefaultMeshInfo());

        mesh->setModelMatrix(oldmesh->getModelMatrix());
        mesh->setWorldMatrix(oldmesh->getWorldMatrix());
        mesh->copyMetaDataFrom(*oldmesh);

        // We can share the buffers here since we won't ever change them in this processor
        // and this is the only place with a non-const versions.
        for (const auto& [info, buff] : oldmesh->getIndexBuffers()) {
            mesh->addIndices(info, buff);
        }

        for (const auto& [info, buff] : oldmesh->getBuffers()) {
            if (info.type == BufferType::ColorAttrib &&
                buff->getDataFormat()->getId() == DataFormat<vec4>::id()) {
                const auto newColors = std::make_shared<BufferRAMPrecision<vec4>>(
                    std::vector<vec4>(buff->getSize(), color));
                mesh->addBuffer(info, std::make_shared<Buffer<vec4>>(newColors));
            } else {
                mesh->addBuffer(info, buff);
            }
        }

        return mesh;
    };

    const bool stateChange = isoValue_.isModified() || invertIso_.isModified() || encloseSurface_.isModified();

    if (stateChange) {
        mesh_ = computeSurface(meshColor_.get(), inport_.getData());
    } else if (inport_.isChanged()) {
        mesh_ = computeSurface(meshColor_.get(), inport_.getData()); 
    } else if (meshColor_.isModified()) {
        mesh_ = changeColor(meshColor_.get(), mesh_);
    }

    outport_.setData(mesh_);
}

}  // namespace inviwo
