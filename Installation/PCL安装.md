&emsp;
# PCL 安装

- 官方网站：https://pointclouds.org/
- GitHub：https://github.com/PointCloudLibrary/pcl
- 安装指南：https://pcl.readthedocs.io/projects/tutorials/en/master/compiling_pcl_posix.html#compiling-pcl-posix


## 2 安装依赖项

>安装依赖项
```c++
sudo apt update
sudo apt-get install -y git build-essential linux-libc-dev
sudo apt-get install -y cmake cmake-gui 
sudo apt-get install -y libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install -y mpi-default-dev openmpi-bin openmpi-common
sudo apt-get install -y libpcap-dev
sudo apt-get install -y libflann1.9 libflann-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libboost-all-dev
sudo apt-get install -y vtk6 libvtk6.3 libvtk6-dev libvtk6.3-qt libvtk6-qt-dev 
sudo apt-get install -y libqhull* libgtest-dev
sudo apt-get install -y freeglut3-dev pkg-config
sudo apt-get install -y libxmu-dev libxi-dev 
sudo apt-get install -y mono-complete
sudo apt-get install -y libopenni-dev libopenni2-dev
```

```c++
// 中途可能有些漏的依赖项需要安装
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt update
sudo apt install-y libpcl-dev
```

```c++
sudo apt install -y pcl-tools
```