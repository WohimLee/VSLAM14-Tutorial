&emsp;
# g2o 安装
## 1 下载 g2o（20170730_git）
- 官方网站：https://openslam-org.github.io/g2o.html
- GitHub地址：https://github.com/RainerKuemmerle/g2o
- 官方文档：http://g2o.xuezhisd.top/index.html


&emsp;
## 2安装依赖项

```
sudo apt install -y libqt4-dev 

sudo apt install -y libqglviewer-dev 
sudo apt install -y libsuitesparse-dev 
sudo apt install -y libcxsparse3.1.2 
sudo apt install -y libcholmod[TAB键] # 我的是2.1.2 和 3
sudo apt install -y uuid-dev

sudo apt install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
```


&emsp;
## 3 编译
在库解压后的库文件夹下打开终端
```
cd [g2o文件夹]
mkdir build
cd build
```

```
cmake ..
make -j7
```

如果装了 Anaconda 会报错
>解决方法
```c++
locate uuid | grep anaconda3/lib
// 删除删除 anaconda3/lib 下所有 libuuid库文件
rm -f libuuid*
```

&emsp;
## 4 安装
```c++
make install
```
