
ROOT := /usr/include/eigen3/

Eigen_INCLUDE := $(ROOT)
Eigen_LIB_DIR := 
Eigen_LIBS := $(wildcard $(Eigen_LIB_DIR)/*.a $(Eigen_LIB_DIR)/*.so)
Eigen_LIBS := $(basename $(notdir $(Eigen_LIBS)))
Eigen_LIBS := $(patsubst lib%,%,$(Eigen_LIBS))