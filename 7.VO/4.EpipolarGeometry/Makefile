include mks/OpenCV.mk
include mks/Eigen.mk
include mks/DBoW2.mk

MAIN := ./Examples/example1.cpp
TARGET := $(basename $(notdir $(MAIN)))
ORB := lib/libSLAM.so


cpp_srcs := $(shell find src -name "*.cpp")
cpp_objs := $(patsubst src/%.cpp,objs/%.o,$(cpp_srcs))

include_dirs := include $(Eigen_INCLUDE) $(OpenCV_INCLUDE) $(DBoW2_INCLUDE)
linking_libs := SLAM $(OpenCV_LIBS) $(DBoW2_LIBS)
library_dirs := lib $(OpenCV_LIB_DIR) $(DBoW2_LIB_DIR)


I_options := $(include_dirs:%=-I%)
l_options := $(linking_libs:%=-l%)
L_options := $(library_dirs:%=-L%)
r_options := $(library_dirs:%=-Wl,-rpath=%)


compile_options := -std=c++11 -fPIC -w $(I_options)
linking_options := $(l_options) $(L_options) $(r_options)

# Compile ORB lib
objs/%.o : src/%.cpp
	@echo [INFO]: Compile $^ to $@
	@mkdir -p $(dir $@)
	@g++ -c $< -o $@ $(compile_options)

$(ORB) : $(cpp_objs)
	@echo [INFO]: Link $^ to $@
	@mkdir -p $(dir $@)
	@g++ -shared $^ -o $@


# Compile example
objs/$(TARGET).o : $(MAIN) 
	@echo [INFO]: Compile $< to $@
	@mkdir -p $(dir $@)
	@g++ -c $< -o $@ $(compile_options)

Examples/$(TARGET) : objs/$(TARGET).o $(ORB)
	@echo [INFO]: Link $^ to $@
	@mkdir -p $(dir $@)
	@g++ $< -o $@ $(linking_options)





run : Examples/$(TARGET)
	@./$<

clean :
	@rm -rf objs Examples/$(TARGET)

debug :
	@echo $(DBoW2_LIBS)

