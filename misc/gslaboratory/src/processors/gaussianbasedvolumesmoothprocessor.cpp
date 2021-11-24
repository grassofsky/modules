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
 *V
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

#include <inviwo/gslaboratory/processors/gaussianbasedvolumesmoothprocessor.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo GaussianBasedVolumeSmoothProcessor::processorInfo_{
    "org.inviwo.GaussianBasedVolumeSmoothProcessor",      // Class identifier
    "Gaussian Based Volume Smooth Processor",                // Display name
    "Volume Operation",              // Category
    CodeState::Experimental,  // Code state
    Tags::CPU,               // Tags
};

const ProcessorInfo GaussianBasedVolumeSmoothProcessor::getProcessorInfo() const { return processorInfo_; }

GaussianBasedVolumeSmoothProcessor::GaussianBasedVolumeSmoothProcessor()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    , gaussianProperty_("gaussianProperty", "GaussianProperty")
    , gaussianKernelDimension_("gaussianKernelDimension", "KernelDimension", 3, 3, 9, 2) {

    addPort(inport_);
    addPort(outport_);

    addProperty(gaussianProperty_);
    addProperty(gaussianKernelDimension_);
}

void GaussianBasedVolumeSmoothProcessor::process() {
    util::log(IVW_CONTEXT, "start processing", LogLevel::Info, LogAudience::User);
    auto volume = inport_.getData();

    if (gaussianProperty_.isModified() || gaussianKernelDimension_.isModified()) {
        calculateKernel();
    }

    auto result = volume->getRepresentation<VolumeRAM>()->dispatch<std::shared_ptr<Volume>>(
        [&](auto vrprecision) {
            using ValueType = util::PrecisionValueType<decltype(vrprecision)>;
            const auto dims = vrprecision->getDimensions();
            const ValueType* srcData = vrprecision->getDataTyped();
            using TResult = typename util::same_extent<ValueType, float>::type;

            auto dstVol = std::make_shared<VolumeRAMPrecision<TResult>>(
                dims, volume->getSwizzleMask(), volume->getInterpolation(), volume->getWrapping());
            TResult* dstData = dstVol->getDataTyped();

            int dimSize = static_cast<int>(glm::compMul(dims));
            const util::IndexMapper3D index{dims};
            auto ivecdims = static_cast<ivec3>(dims);
            #pragma omp parallel for
            for (int i = 0; i < dimSize; ++i) {
                size3_t curPos = index(i);
                TResult gaussianValue = static_cast<TResult>(0.0f);
                for (size_t j = 0, sizej = kernelOffsets_.size(); j < sizej; ++j) {
                    auto& kernelOffset = kernelOffsets_[j];
                    auto& kernelValue = kernelValues_[j];
                    auto tempPos = static_cast<ivec3>(curPos) + kernelOffset;
                    // Out of volume box region
                    if (glm::all(glm::greaterThanEqual(tempPos, ivec3(0))) &&
                        glm::all(glm::lessThan(tempPos, ivecdims))) {
                        auto tempIndex = index(tempPos);
                        gaussianValue += kernelValue * static_cast<TResult>(srcData[tempIndex]);
                    }
                }
                dstData[i] = gaussianValue;
            }

            auto vol = std::make_shared<Volume>(dstVol);
            vol->setBasis(volume->getBasis());
            vol->setOffset(volume->getOffset());
            vol->copyMetaDataFrom(*volume.get());
            return vol;
        });
    util::log(IVW_CONTEXT, "finish processing", LogLevel::Info, LogAudience::User);

    outport_.setData(result);
}

void GaussianBasedVolumeSmoothProcessor::calculateKernel() {
    int kernelOneDim = gaussianKernelDimension_.get();
    size3_t kernelCenter(kernelOneDim / 2, kernelOneDim / 2, kernelOneDim / 2);
    kernelOffsets_.clear();
    kernelValues_.clear();
    for (int z = 0; z < kernelOneDim; ++z) {
        for (int y = 0; y < kernelOneDim; ++y) {
            for (int x = 0; x < kernelOneDim; ++x) {
                dvec3 kernelOffset = dvec3(x, y, z) - dvec3(kernelCenter);
                kernelOffsets_.push_back(kernelOffset);
                kernelValues_.push_back(static_cast<float>(gaussianProperty_.evaluate(kernelOffset)));
            }
        }
    }
}
}  // namespace inviwo
