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

#include <inviwo/gslaboratory/processors/maskselectionprocessor.h>

#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MaskSelectionProcessor::processorInfo_{
    "org.inviwo.MaskSurfaceExtractionProcessor",      // Class identifier
    "Mask Selection Processor",                // Display name
    "Volume Operation",              // Category
    CodeState::Experimental,  // Code state
    Tags::CPU,               // Tags
};
const ProcessorInfo MaskSelectionProcessor::getProcessorInfo() const { return processorInfo_; }

MaskSelectionProcessor::MaskSelectionProcessor()
    : Processor()
    , inport_("inputVolume")
    , outport_("outportVolume")
    , selectedValue_("selectedValue", "selected mask labels", {0, 0, 0, 0}) {

    addPort(inport_);
    addPort(outport_);

    addProperty(selectedValue_);
}

void MaskSelectionProcessor::process() { 
    auto volume = inport_.getData(); 
    auto result = volume->getRepresentation<VolumeRAM>()->dispatch<std::shared_ptr<Volume>>(
        [&](auto vrprecision) {
            using ValueType = util::PrecisionValueType<decltype(vrprecision)>;
            const auto dims = vrprecision->getDimensions();
            const ValueType* srcData = vrprecision->getDataTyped();
            using TResult = typename util::same_extent<ValueType, float>::type;

            auto dstVol = std::make_shared<VolumeRAMPrecision<TResult>>(
                dims, volume->getSwizzleMask(), volume->getInterpolation(), volume->getWrapping());
            TResult* dstData = dstVol->getDataTyped();

            auto selectedValue = selectedValue_.get();
            std::vector<ValueType> selectedMask;
            if (selectedValue.x != 0)
                selectedMask.push_back(static_cast<ValueType>(selectedValue.x));
            if (selectedValue.y != 0)
                selectedMask.push_back(static_cast<ValueType>(selectedValue.y));
            if (selectedValue.z != 0)
                selectedMask.push_back(static_cast<ValueType>(selectedValue.z));
            if (selectedValue.w != 0)
                selectedMask.push_back(static_cast<ValueType>(selectedValue.w));
            std::transform(
                srcData, srcData + glm::compMul(dims), dstData, [&selectedMask](auto& v) {
                    if (!selectedMask.empty()) {
                        auto iter = std::find(selectedMask.begin(), selectedMask.end(), v);
                        if (iter != selectedMask.end()) {
                            return static_cast<TResult>(1.0f);
                        }
                    } 

                    return static_cast<TResult>(0.0f);
                });

            auto vol = std::make_shared<Volume>(dstVol);
            vol->setBasis(volume->getBasis());
            vol->setOffset(volume->getOffset());
            vol->copyMetaDataFrom(*volume.get());
            return vol;
        }
    );

    result->dataMap_.dataRange = dvec2(0,1);
    result->dataMap_.valueRange = dvec2(0,1);

    outport_.setData(result);
}

}  // namespace inviwo
