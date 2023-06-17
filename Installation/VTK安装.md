&emsp;
# VTK 安装
官方安装文档：https://github.com/Kitware/VTK/blob/master/Documentation/dev/build.md#building

## 1 下载VTK（7.1.1）
或8.2



&emsp;
## 2 安装依赖项
```shell
sudo apt install -y \
build-essential mesa-common-dev mesa-utils freeglut3-dev
```

&emsp;
## 3 编译安装
老套路
```cmake
cmake -D CMAKE_INSTALL_PREFIX=/home/liheqian/AZen/3rdParty/VTK-7.1.1 ..
cmake -D CMAKE_INSTALL_PREFIX=/home/liheqian/AZen/3rdParty/Pangolin0.6 ..
```
