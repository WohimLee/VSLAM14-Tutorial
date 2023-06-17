
cpps := $(shell find src -name "*.cpp")
objs := $(patsubst src/%.cpp,objs/%.o,$(cpps))

libraries := Sophus \
			 opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs \
			 boost_system

include_paths := /usr/include/eigen3 \
				 /home/liheqian/datav/SLAM/3rdparty/Sophus \
				 /home/liheqian/datav/3rdparty/opencv/include/opencv4/ \
				 /usr/include/pcl-1.8 \
				 /usr/include/vtk-6.3


library_paths := /home/liheqian/datav/SLAM/3rdparty/Sophus/svs_build \
				 /home/liheqian/datav/3rdparty/opencv/lib  \
				 /usr/lib/x86_64-linux-gnu/

options := -std=c++11 -w -g -m64

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

workspace/exec : $(objs)
	@echo Link $^ to $@
	@mkdir -p $(dir $@)
	@g++ $^ -o $@ $(linking_options)

run : workspace/exec
	@./$<



debug :
	@echo $(compile_options)
	@echo $(objs)

clean:
	@rm -rf workspace/exec objs

.PHONY : clean debug run
