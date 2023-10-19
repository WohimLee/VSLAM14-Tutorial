ROOT := /home/nerf/datav/SLAM/ORB_SLAM3/Thirdparty

Sophus_INCLUDE := $(ROOT)/Sophus
Sophus_LIB_DIR := 
Sophus_LIBS := $(wildcard $(Sophus_LIB_DIR)/*.a $(Sophus_LIB_DIR)/*.so)
Sophus_LIBS := $(basename $(notdir $(Sophus_LIBS)))
Sophus_LIBS := $(patsubst lib%,%,$(Sophus_LIBS))