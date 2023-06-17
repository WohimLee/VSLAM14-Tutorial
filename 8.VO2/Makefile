
PROJECT := 8.5.2_derect_sparse
SRC_DIR := src
MAIN := main.cpp
cpps := $(shell find $(SRC_DIR) -name "$(PROJECT).cpp") $(SRC_DIR)/$(MAIN) 
objs := $(patsubst src/%.cpp,objs/%.o,$(cpps))


base_libs := boost_system pcl_io pcl_common \
			pthread glog cholmod lapack cxsparse 


include_paths := /home/liheqian/datav/SLAM/test/src\
				 /usr/include/eigen3 \
				 /home/liheqian/datav/SLAM/3rdparty/Sophus \
				 /home/liheqian/datav/3rdparty/opencv/include/opencv4/ \
				 /usr/include/pcl-1.8 \
				 /usr/include/vtk-6.3 \
				 /home/liheqian/datav/SLAM/3rdparty/ceres-solver-2.1.0/include/ \
				 /home/liheqian/datav/SLAM/3rdparty/ceres-solver-2.1.0/build/include/ \
				 /home/liheqian/datav/3rdparty/sourcecode/g2o/ \
				 /home/liheqian/datav/3rdparty/sourcecode/g2o/build/ \
				 /usr/include/suitesparse/

base_lib_paths := /usr/lib/x86_64-linux-gnu 
				 

3rd_libs_path  := /home/liheqian/datav/SLAM/3rdparty/Sophus/svs_build \
				  /home/liheqian/datav/SLAM/3rdparty/ceres-solver-2.1.0/build/lib \
				  /home/liheqian/datav/3rdparty/sourcecode/g2o/lib \
				  /home/liheqian/datav/3rdparty/opencv/lib

library_paths  := $(base_lib_paths) $(3rd_libs_path)


so_libs := $(shell find $(3rd_libs_path) -maxdepth 1 -name "lib*.so")
so_libs := $(notdir $(so_libs))
so_libs := $(so_libs:lib%.so=%)
so_libs := $(sort $(so_libs))

a_libs := $(shell find $(3rd_libs_path) -maxdepth 1 -name "lib*.a")
a_libs := $(notdir $(a_libs))
a_libs := $(a_libs:lib%.a=%)
a_libs := $(sort $(a_libs))

libraries := $(base_libs) $(so_libs) $(a_libs)
libraries := $(sort $(libraries))

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
	@mkdir -p ./output
	@./$<


debug :
	@echo $(L_options)


clean:
	@rm -rf workspace/$(PROJECT) objs output
	

.PHONY : clean debug run
