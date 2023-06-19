&emsp;
# Coordinate Systems

坐标系的定义很多，还分左手坐标系和右手坐标系

>世界坐标系（World Coordinate System）
- $[x, y, z]$
- [right, up, forward]
- 一种通常用于机器人运动规划和控制的全局坐标系，通常用来描述机器人和其它物体在三维空间中的位置和方向信息

>相机坐标系（Camera Coordinate System）
- $[x, -y, -z]$
- [right, down, backward]
- 一种以相机光轴为 z 轴、以相机成像平面上的一个点为原点的坐标系，可以用来描述相机在三维空间中的位置和方向等信息

>基座坐标系（Base Coordinate System/Device Reference Base, DRB）
- [x, -y, -z]
- [right, down, backward]
- 一种相对于机器人基座固定的坐标系，通常用来描述机器人基座在三维空间中的位置和方向信息


>工具坐标系（Tool Coordinate System/Robotics Device Frame, RDF）
- [x, -y, z]
- [right, down, forward]
- 一种相对于机器人末端执行器固定的坐标系，可以用来描述机器人末端执行器在三维空间中的位置和姿态信息

>用户坐标系（User Coordinate System/Robot User Frame, RUF）
- [x, y, -z]
- [right, up, backward]
- 一种相对于机器人末端执行器可以自由定义的坐标系，通常用来描述机器人末端执行器上的工具或工件在三维空间中的位置和方向信息

>关节坐标系（Joint Coordinate System）
- [x, -y, z]
- [right, down, forward]
- 一种相对于机器人关节角度定义的坐标系，通常用来描述机器人关节运动和姿态信息

>末端执行器局部坐标系（Local Reference Frame）
- [right, down, forward]
- 一种相对于机器人末端执行器固定的局部坐标系，通常用来描述机器人末端执行器上的局部特征和位置信息

>惯性导航坐标系（Inertial Navigation Coordinate System）
- [right, forward, up]
- 一种相对于地球固定的坐标系，通常用于机器人的导航和定位