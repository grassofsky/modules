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

#ifndef IVW_MESHOTHERUTILS_H
#define IVW_MESHOTHERUTILS_H

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
#include <warn/pop>
#include "openmeshconverters.h"

#ifdef _USE_MATH_DEFINES_WAS_DEFINED
#undef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES_WAS_DEFINED
#endif

namespace inviwo {

namespace openmeshutil {

/**
 * Utility function to calculate volume of mesh.
 * 
 * Ref paper is: Cha Zhang and Tsuhan Chen, "Efficient feature extraction for 2D/3D objects in mesh representation,"
 * Proceedings 2001 International Conference on Image Processing (Cat. No.01CH37205),
 * Thessaloniki, Greece, 2001, pp. 935-938 vol.3, doi: 10.1109/ICIP.2001.958278.
 * 
 * @param mesh An OpenMesh mesh (see fromInviwo(...) and ::toInviwo(...))
 * @return mesh volume
 */
template <typename OMesh>
double calculateTriMeshVolume(const OMesh& mesh) {
    double volume = 0.0;

    std::vector<OMesh::Point> ptsOnFace;
    for (auto face : mesh.faces()) {
        ptsOnFace.clear();
        for (auto vertex : mesh.fv_range(face)) {
            ptsOnFace.push_back(mesh.point(vertex));
        }
        if (ptsOnFace.size() != 3) {
            throw std::runtime_error("The face for volume calculation must be triangle");
        }
        auto& pt1 = ptsOnFace[0];
        auto& pt2 = ptsOnFace[1];
        auto& pt3 = ptsOnFace[2];

        volume += 1.0 / 6.0 *
                   (-pt3[0] * pt2[1] * pt1[2] + pt2[0] * pt3[1] * pt1[2] + pt3[0] * pt1[1] * pt2[2] -
                    pt1[0] * pt3[1] * pt2[2] - pt2[0] * pt1[1] * pt3[2] + pt1[0] * pt2[1] * pt3[2]);
    }
    return volume;
}

}  // namespace openmeshutil

}  // namespace inviwo

#endif  // IVW_MESHOTHERUTILS_H
