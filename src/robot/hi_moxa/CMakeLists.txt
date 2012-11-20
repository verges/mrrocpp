add_library(hi_moxa
	hi_moxa.cc
	serialcomm.cpp
	nfv2.c
	mycrc.c
)

install(TARGETS hi_moxa DESTINATION lib)
