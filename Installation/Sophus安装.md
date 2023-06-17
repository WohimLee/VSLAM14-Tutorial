&emsp;
# Sophus 安装
>准备工作
```shell
# 安装 cmake make
sudo apt update
sudo apt install -y cmake make
```

&emsp;
## 1 解压 Sophus.tar.gz

```shell
cd 3rdparty
tar -xzvf Sophus.tar.gz
```

&emsp;
## 2 安装
### 2.1 创建 build 文件夹
>在 Terminal 输入
```shell
cd Sophus
mkdir svs_build
cd svs_build
```
### 2.2 编译 Sophus
>在 Terminal 输入
```shell
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/svslocal
```
高博附带的 3rdparty 内的 Sophus 编译会报错

<div align="center">
    <image src="./imgs/Sophus/Sophus-2.png" width = 700>
</div>
&emsp;

修改下面文件

<div align="center">
    <image src="./imgs/Sophus/Sophus-1.png" width = 500>
</div>
&emsp;

>在 Terminal 输入
```shell
make -j4
make install
```
