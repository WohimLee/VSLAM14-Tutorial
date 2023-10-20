
ROOT := /home/nerf/datav/3rdParty/opencv-4.5.1

OpenCV_INCLUDE := $(ROOT)/include/opencv4
OpenCV_LIB_DIR := $(ROOT)/lib
OpenCV_LIBS := $(wildcard $(OpenCV_LIB_DIR)/*.a $(OpenCV_LIB_DIR)/*.so)
OpenCV_LIBS := $(basename $(notdir $(OpenCV_LIBS)))
OpenCV_LIBS := $(patsubst lib%,%,$(OpenCV_LIBS))
