############################################################################
#
#   aperocopter coaxial per-blade mixer
#
############################################################################

# mixer.hpp is shared with libplant in the sibling aperocopter repo (header-only, no Eigen)
set(APEROCOPTER_DIR ${PX4_SOURCE_DIR}/../aperocopter CACHE PATH "aperocopter repo")

px4_add_module(
	MODULE modules__coax_mixer
	MAIN coax_mixer
	SRCS
		CoaxMixer.cpp
		CoaxMixer.hpp
	INCLUDES
		${APEROCOPTER_DIR}/tools/plant_cpp/include
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
	)
