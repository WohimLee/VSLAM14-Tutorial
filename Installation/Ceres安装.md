&emsp;
# Ceres 安装
## 1 下载Ceres（1.14.0）
- 官方网站：http://ceres-solver.org/
- GitHub地址：https://github.com/ceres-solver/ceres-solver


&emsp;
## 2安装依赖项
```c++
sudo apt install -y liblapack-dev libsuitesparse-dev  libgflags-dev libgoogle-glog-dev libgtest-dev

sudo apt install -y liblapacke-dev

sudo apt install -y libtbb-dev

sudo apt-get install libatlas-base-dev

sudo apt install -y libcxsparse3.1.2
```

可能会出现无法定位libcxsoarse3.1.2的问题：

>解决方法
- 第一步，打开sources.list
    ```c++
    sudo gedit /etc/apt/sources.list
    ```
- 第二步，将下面的源粘贴到最上方 sources.list
    ```c++
    deb http://cz.archive.ubuntu.com/ubuntu trusty main universe 
    ```
- 第三步，更新源
    ```c++
    sudo apt-get update
    ```
- 第四步，重新输入依赖项安装命令安装依赖项
    ```
    sudo apt install -y libcxsparse3.1.2
    ```

&emsp;
## 3 编译
在库解压后的库文件夹下打开终端
```
cd ceres-solver-1.14.0
mkdir build
cd build
```

```
cmake ..
make -j7
```

&emsp;
## 4 安装
```c++
make install
```
