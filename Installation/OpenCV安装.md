&emsp;
# 安装 OpenCV
- 环境：Ubuntu 18.04

## 1 安装 git
```shell
sudo apt update  # 只检查，不更新（已安装的软件包是否有可用的更新，给出汇总报告）
sudo apt upgrade # 更新已安装的软件包
sudo apt install -y git
```

&emsp;
## 2 下载 OpenCV 源码(3.4.8)

（1）用git命令下载OpenCV
```c++
git clone https://gitee.com/yxd.osc.com/opencv.git
```

（2）安装software-properties-common
```c++
sudo apt install -y -f software-properties-common
```

（3）检查是否包含 viz 模块
<div align="center">
    <image src="./imgs/opencv.png" width = 500>
</div>
&emsp;
没有的话需要自己下载

- 扩展模块下载地址：https://gitcode.net/mirrors/opencv/opencv_contrib

&emsp;
## 3 安装依赖项
>添加仓库
- 如果你用的是国内的源，应该就不用这个操作，否则会报错
```c++
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu \
xenial-security main"

sudo apt update
```

直接执行下面这个，其它都弄到这里了
```
sudo apt install -y build-essential \
libjasper1 libjasper-dev \
cmake libgtk2.0-dev pkg-config libavcodec-dev \
libavformat-dev libswscale-dev \
python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev \
libpng-dev libdc1394-22-dev \
webp libwebp-dev
```

&emsp;
## 4 安装 VTK(7.1.1)

&emsp;
## 5 生成安装文件(时间比较久)
到 opencv/CMakeList.txt 文件的目录下执行建 build 文件夹
```
mkdir build
cd build
```

执行
```python
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=[自定义最终安装路径] ..

# 示例一（无UI）
# 注意是最终文件夹，它不会另外创建文件夹放文件示例
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/home/liheqian/datav/3rdparty/opencv ..

# 示例二（图形界面）
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/home/liheqian/datav/3rdParty/opencv3.2.0 \
      -D WITH_TBB=ON \
      -D OPENCV_GENERATE_PKGCONFIG=YES \
      -D BUILD_NEW_PYTHON_SUPPORT=ON \
      -D WITH_V4L=ON \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D BUILD_EXAMPLES=ON \
      -D WITH_QT=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_GTK=ON \
      -D BUILD_TIFF=ON \
      -D WITH_VTK=ON \
      -D VTK_DIR=/home/liheqian/datav/3rdparty/sourcecode/VTK-7.1.1/build \
      -D WITH_OPENGL=ON  ..

# 示例三：简单安装
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/home/liheqian/AZen/3rdParty/opencv3.4.8 ..
```

在build文件夹下执行
```
make -j7
```

&emsp;
## 6 安装 OpenCV
在 build 目录下执行
```
make install
```


&emsp;
## 7 其它
```c++
sudo apt install -y libopencv-viz-dev
sudo ldconfig
```




