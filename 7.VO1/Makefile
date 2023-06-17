
PROJECT := 7.8_pose_estimation_3d2d
SRC_DIR := src
MAIN := main.cpp
cpps := $(shell find $(SRC_DIR) -name "$(PROJECT).cpp") $(SRC_DIR)/$(MAIN) 
objs := $(patsubst src/%.cpp,objs/%.o,$(cpps))


# libraries := Sophus \
# 			 opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs \
# 			 opencv_ml opencv_objdetect opencv_features2d opencv_gapi\
# 			 opencv_calib3d \
# 			 boost_system pcl_io pcl_common \
# 			 ceres pthread glog cholmod lapack cxsparse \
# 			 g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension

			 

include_paths := /usr/include/eigen3 \
				 /home/liheqian/datav/SLAM/3rdparty/Sophus \
				 /home/liheqian/datav/3rdparty/opencv/include/opencv4/ \
				 /usr/include/pcl-1.8 \
				 /usr/include/vtk-6.3 \
				 /home/liheqian/datav/SLAM/3rdparty/ceres-solver-2.1.0/include/ \
				 /home/liheqian/datav/SLAM/3rdparty/ceres-solver-2.1.0/build/include/ \
				 /home/liheqian/datav/3rdparty/sourcecode/g2o/ \
				 /home/liheqian/datav/3rdparty/sourcecode/g2o/build/ \
				 /usr/include/suitesparse/

library_paths := /home/liheqian/datav/SLAM/3rdparty/Sophus/svs_build \
				 /home/liheqian/datav/3rdparty/opencv/lib  \
				 /usr/lib/x86_64-linux-gnu \
				 /home/liheqian/datav/SLAM/3rdparty/ceres-solver-2.1.0/build/lib \
				 /home/liheqian/datav/3rdparty/sourcecode/g2o/lib



so_libs := $(shell find $(library_paths) -maxdepth 1 -name "lib*.so")
so_libs := $(notdir $(so_libs))
so_libs := $(so_libs:lib%.so=%)

a_libs := $(shell find $(library_paths) -maxdepth 1 -name "lib*.a")
a_libs := $(notdir $(a_libs))
a_libs := $(a_libs:lib%.a=%)

libraries := $(so_libs) $(a_libs)

options :=  -w -g -m64 # -std=c++11

I_options := $(include_paths:%=-I%)
l_options := $(libraries:%=-l%)
L_options := $(library_paths:%=-L%)
r_options := $(library_paths:%=-Wl,-rpath=%)

compile_options := $(options) $(I_options)
linking_options := $(l_options) $(L_options) $(r_options)

objs/%.o : src/%.cpp
	@echo Compile $(^)
	@mkdir -p $(dir $@)
	@g++ -c $^ -o $@ $(compile_options)

workspace/$(PROJECT)/exec : $(objs)
	@echo Link $^ to $@
	@mkdir -p $(dir $@)
	@g++ $^ -o $@ $(linking_options)

run : workspace/$(PROJECT)/exec
	@./$<


debug :
	@echo $(objs)


clean:
	@rm -rf workspace/$(PROJECT)/exec objs
	

.PHONY : clean debug run
