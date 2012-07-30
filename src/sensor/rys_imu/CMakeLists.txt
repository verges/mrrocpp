add_library(rys_imu
	edp_s.cc
	imu.cpp
)

install(TARGETS rys_imu DESTINATION lib)