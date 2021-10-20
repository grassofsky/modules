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

#pragma once

#include <inviwo/openmesh/openmeshmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/ports/imageport.h>

namespace inviwo {

/** \docpage{org.inviwo.HCLaplacianSmootherProcessor, HCLaplacian Smoother Processor}
 * ![](org.inviwo.HCLaplacianSmootherProcessor.png?classIdentifier=org.inviwo.HCLaplacianSmootherProcessor)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __inmeshes__ Input multi meshes.
 *
 * ### Outports
 *   * __outmeshes__ Smoothed multi meshes.
 *
 * ### Properties
 *   * __Number of iteration in smoothing__ The number of iteration will be executed in smoothing.
 *   * __alpha__ original position influence factor
 *   * __beta__ current position influence factor
 *   * __enable calculate volume__ The flag to control if calculating mesh volume
 */
class IVW_MODULE_OPENMESH_API HCLaplacianSmootherProcessor : public Processor {
public:
    HCLaplacianSmootherProcessor();
    virtual ~HCLaplacianSmootherProcessor() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    MeshFlatMultiInport inmeshes_{"inmeshes"};
    DataOutport<std::vector<std::shared_ptr<Mesh>>> outmeshes_{"outmeshes"};

    IntProperty iteratorNumber_{
        "iteratorNumber_", "Number of iteration in smoothing", 5, 0, 100, 1};
    FloatProperty alpha_{"alpha_", "original position influence factor", 0.f, 0.f, 1.f, 0.001f};
    FloatProperty beta_{"beta_", "current position influence factor", 0.f, 0.f, 1.f, 0.001f};
    BoolProperty enableCalculateVolume_{"enableCalculateVolume_", "if calculating volume of mesh",
                                        false};
};

}  // namespace inviwo
