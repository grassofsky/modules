ivw_module(GSLaboratory)

set(HEADER_FILES
    include/inviwo/gslaboratory/gslaboratorymodule.h
    include/inviwo/gslaboratory/gslaboratorymoduledefine.h
    include/inviwo/gslaboratory/processors/gaussianbasedvolumesmoothprocessor.h
    include/inviwo/gslaboratory/processors/maskselectionprocessor.h
    include/inviwo/gslaboratory/processors/morphologybasedvolumesmoothprocessor.h
    include/inviwo/gslaboratory/processors/shaderpreprocesstofile.h
    include/inviwo/gslaboratory/processors/singlesurfaceextractionprocessor.h
    include/inviwo/gslaboratory/utils/volumesmoother.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/gslaboratorymodule.cpp
    src/processors/gaussianbasedvolumesmoothprocessor.cpp
    src/processors/maskselectionprocessor.cpp
    src/processors/morphologybasedvolumesmoothprocessor.cpp
    src/processors/shaderpreprocesstofile.cpp
    src/processors/singlesurfaceextractionprocessor.cpp
    src/utils/volumesmoother.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	# Add shaders
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/gslaboratory-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
