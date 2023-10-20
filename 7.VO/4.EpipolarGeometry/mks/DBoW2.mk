ROOT := /home/nerf/datav/SLAM/ORB_SLAM3/Thirdparty

DBoW2_INCLUDE := $(ROOT)/DBoW2
DBoW2_LIB_DIR := $(ROOT)/DBoW2/lib
DBoW2_LIBS := $(wildcard $(DBoW2_LIB_DIR)/*.a $(DBoW2_LIB_DIR)/*.so)
DBoW2_LIBS := $(basename $(notdir $(DBoW2_LIBS)))
DBoW2_LIBS := $(patsubst lib%,%,$(DBoW2_LIBS))