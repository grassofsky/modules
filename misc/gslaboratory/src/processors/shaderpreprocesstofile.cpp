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

#include <inviwo/gslaboratory/processors/shaderpreprocesstofile.h>

#include <fstream>

#include <modules/opengl/shader/shaderobject.h>
#include <modules/opengl/shader/shaderutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ShaderPreprocessToFile::processorInfo_{
    "org.inviwo.ShaderPreprocessToFile",      // Class identifier
    "Shader Preprocess To File",                // Display name
    "Data Output",              // Category
    CodeState::Experimental,  // Code state
    Tags::CPU,               // Tags
};
const ProcessorInfo ShaderPreprocessToFile::getProcessorInfo() const { return processorInfo_; }

ShaderPreprocessToFile::ShaderPreprocessToFile()
    : Processor()
    , fileIn_("filenameIn", "shader in", "", "shader")
    , fileOut_("filenameOut", "shader out (full name)", "", "shader") 
    , execute_("execute", "Shader Preprocess") {

    addProperty(fileIn_);
    addProperty(fileOut_);
    addProperty(execute_);
}

void ShaderPreprocessToFile::process() {
    // outport_.setData(myImage);
    if (execute_.isModified())
    {
        ShaderObject shaderObj(ShaderType::Fragment, utilgl::findShaderResource(fileIn_.get()));
        std::string strShader = shaderObj.print();

        std::ofstream ofile(fileOut_.get());
        ofile << strShader;
        ofile.close();
    }
}

}  // namespace inviwo
