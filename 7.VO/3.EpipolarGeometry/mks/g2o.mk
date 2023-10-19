ROOT := /home/nerf/datav/SLAM/ORB_SLAM3/Thirdparty

g2o_INCLUDE := $(ROOT)/g2o
g2o_LIB_DIR := $(ROOT)/lib
g2o_LIBS := $(wildcard $(g2o_LIB_DIR)/*.a $(g2o_LIB_DIR)/*.so)
g2o_LIBS := $(basename $(notdir $(g2o_LIBS)))
g2o_LIBS := $(patsubst lib%,%,$(g2o_LIBS))