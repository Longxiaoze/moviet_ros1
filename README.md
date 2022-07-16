# moveit_ros1

**说明：使用ros melodic和moveit对cr5机械臂进行仿真实验，自己在参考代码基础上实现的功能见[1、说明](https://github.com/Longxiaoze/moviet_ros1#1%E8%AF%B4%E6%98%8E)**

**如果是第一次使用，请先按照[二、安装](https://github.com/Longxiaoze/moviet_ros1#%E4%BA%8C%E5%AE%89%E8%A3%85)进行仿真环境安装，然后将下述两个文件进行替换**

~/catkin_ws/src/CR5_ROS/dobot_description/urdf/cr5_robot.urdf

~/ws_moveit/src/moveit_tutorials/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial_cr5_add_3d_zivid_point_check_load_mesh_tf_new.py

## 0、运行

**终端一：启动cr5（rviz仿真环境）**

```plain
cd ~/catkin_ws/
source devel/setup.bash
roslaunch dobot_moveit demo.launch
```

**终端二：控制机械臂运动**

```plain
cd ~/ws_moveit
source devel/setup.bash
rosrun moveit_tutorials move_group_python_interface_tutorial_cr5_add_3d_zivid_point_check_load_mesh_tf_new.py
```
## 1、说明：

**move_group_python_interface_tutorial_cr5_add_3d_zivid_point_check_load_mesh_tf_new.py**

### （1）相关参数说明

|参数名称|说明|备注|
|:----|:----|:----|
|**self.pose_start**<br>|第一次到达点的位置|pose都是xyz和四元数的形式|
|**self.pose_goal**<br>|目标到达点的位置（有移动函数）|    |
|**self.transformPose_name_end_link**<br>|移动函数中转换的坐标名称|    |
|**self.transformPose_name_end_link_begin**<br>|移动函数中转换到哪个坐标系下的坐标名称|    |
|**self.add_mesh_pose**<br>|转向架网格文件添加的位姿|    |
|**self.file_stl_name**<br>|转向架网格文件的名称|注意不要有中文路径|
|**self.add_agv_pose**<br>|底盘模型添加的位姿|    |
|**self.file_agv_name**<br>|底盘模型的名称|注意不要有中文路径|
|**self.add_wall_left_pose**<br>|添加墙左边的位姿|    |
|**self.add_wall_right_pose**<br>|添加墙右边的位姿|    |
|**self.add_wall_size**<br>|添加墙的大小|[厚度，长度，高度]|
|**self.add_ground_pose**<br>|模拟添加底盘的地面的位姿|    |

### （2）相关函数说明：

|函数名称|说明|备注|
|:----|:----|:----|
|**tf_listener_get_pose**|给定位姿和距离，求解沿着最后一个link的z轴方向移动给定距离后的位姿|    |
|**go_to_pose_use_tf**<br>|调用上述函数，如果无法到达，则沿着四元数方向进行移动，求解在该方向上的可到达位置|极限搜索距离1.5m，每次移动0.1（可修改）|
|**add_meshes**<br>|添加3D模型|支持stl/dae/ply等文件格式，不支持pcd格式，转换见二、2.7|
|**add_wall**<br>|添加两侧的墙体，|**目前没有与转向架作关联，后续添加**|
|**go_to_pose_start**<br>|将末端到达指定的pose（起始点）|    |
|**go_to_pose_goal**<br>|将末端到达指定的pose（badcase点，这是go_to_pose_use_tf函数的没有优化的代码）|    |
|**add_ground**<br>|模拟底盘添加地面代码，使机械臂无法到达下方|    |
|**以下暂时并未使用**|    |    |
|**plan_cartesian_path**<br><br>|计划一个连续移动的路径|这三个函数共同完成了机械臂末端进行连续轨迹运动的过程|
|**display_trajectory**<br>|将一个轨迹进行展示|    |
|**execute_plan**<br>|移动一个plan|    |
|**wait_for_state_update**<br>|状态更新函数|    |
|**add_box**<br>|在场景中加入一个box，可以避障|    |
|**attach_box**<br>|将box与link进行连接，使之随着机械臂一起运动|    |
|**detach_box**<br>|将box与link进行分离，使之无法随着机械臂一起运动|    |
|**remove_box**<br>|在场景中移除一个box，如果attach_box必须将box先进行detach再remove|    |
|**add_area**<br>|在场景中添加一个区域进行避障，其实就是修改了add_box函数|    |
|**go_to_joint_state**<br>|根据关节的状态进行机械臂位置的改变|    |

# 一、基础内容的参考网址

[http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html)

[https://github.com/Dobot-Arm/CR_ROS](https://github.com/Dobot-Arm/CR_ROS)

[https://sub.qiduo.eu.org/link/VjrCWucrrEzx6zHa?mu=1](https://sub.qiduo.eu.org/link/VjrCWucrrEzx6zHa?mu=1)

open dae files:

[https://www.cs.unca.edu/~bruce/Fall12/373/glcGuide.html](https://www.cs.unca.edu/~bruce/Fall12/373/glcGuide.html)

# 二、安装

## 1、环境

ubuntu18.04 + melodic + moveit

## 2、melodic

```plain
wget http://fishros.com/install -O fishros && . fishros
```
(rosdep init & rosdep update报错及解决：[https://proudrabbit.gitee.io/%E4%BD%BF%E7%94%A8%E4%BB%A3%E7%90%86%E8%A7%A3%E5%86%B3ros%E5%AE%89%E8%A3%85%E8%BF%87%E7%A8%8B%E4%B8%ADrosdep%20update%E8%BF%9E%E6%8E%A5%E8%B6%85%E6%97%B6%E7%9A%84%E9%97%AE%E9%A2%98.html](https://proudrabbit.gitee.io/%E4%BD%BF%E7%94%A8%E4%BB%A3%E7%90%86%E8%A7%A3%E5%86%B3ros%E5%AE%89%E8%A3%85%E8%BF%87%E7%A8%8B%E4%B8%ADrosdep%20update%E8%BF%9E%E6%8E%A5%E8%B6%85%E6%97%B6%E7%9A%84%E9%97%AE%E9%A2%98.html))终端代理
## 2、moveit

[http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-moveit](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-moveit)

## 3、CR5

[https://github.com/Dobot-Arm/CR_ROS/blob/melodic-devel/README-CN.md](https://github.com/Dobot-Arm/CR_ROS/blob/melodic-devel/README-CN.md)

## 4、zivid-camera

[https://www.google.com.hk/search?q=zivid_ros&oq=zivid_ros&aqs=chrome..69i57j0i30l2j0i8i30.5483j0j7&sourceid=chrome&ie=UTF-8](https://www.google.com.hk/search?q=zivid_ros&oq=zivid_ros&aqs=chrome..69i57j0i30l2j0i8i30.5483j0j7&sourceid=chrome&ie=UTF-8)

[https://github.com/zivid/zivid-ros](https://github.com/zivid/zivid-ros)

[https://zhuanlan.zhihu.com/p/94701620](https://zhuanlan.zhihu.com/p/94701620)

## 5、freecad（转换stp文件为dae文件/）

[https://blog.csdn.net/apple_52810383/article/details/121301485](https://blog.csdn.net/apple_52810383/article/details/121301485)

[https://www.freecadweb.org/downloads.php](https://www.freecadweb.org/downloads.php)

[https://github.com/FreeCAD/FreeCAD/releases/download/0.19.3/FreeCAD_0.19.3-Linux-Conda_glibc2.12-x86_64.AppImage](https://github.com/FreeCAD/FreeCAD/releases/download/0.19.3/FreeCAD_0.19.3-Linux-Conda_glibc2.12-x86_64.AppImage)  （下载地址，最终使用这个导出dae文件）

[https://blog.csdn.net/ouening/article/details/81535182](https://blog.csdn.net/ouening/article/details/81535182) （坑：导出时会报错）

## 6、（转换stl文件为dae文件， file<50MB）

[https://anyconv.com/stl-to-dae-converter/](https://anyconv.com/stl-to-dae-converter/)

## 7、pcd转换为ply

[https://www.pythonheidong.com/blog/article/512463/96c44997fde09987b4e7/](https://www.pythonheidong.com/blog/article/512463/96c44997fde09987b4e7/)

下载：

```python
git clone https://github.com/ryanfb/pcl-tools 
cd pcl-tools 
mkdir build
cd build
cmake ..
make 
./pcd2ply '/home/longxiaoze/文档/cr5_arm_add_3d/02moveit_mapping_2/slam_map/digou.pcd' '/home/longxiaoze/文档/cr5_arm_add_3d/02moveit_mapping_2/slam_map/digou.ply'
```

使用：

```python
/home/longxiaoze/codes/c++codes/pcl-tools/build/pcd2ply pcd文件 输出ply文件
```
## 8、转换ply文件为stl文件

使用工具：meshlab

参考网站：

[https://www.cnblogs.com/jiaxinli/p/12195281.html](https://www.cnblogs.com/jiaxinli/p/12195281.html)

[http://fabacademy.org/archives/2014/tutorials/pointcloudToSTL.html](http://fabacademy.org/archives/2014/tutorials/pointcloudToSTL.html)

操作步骤：

1）打开meshlab

2）导入.ply文件

3）viewer->show layer dialog打开

4）计算顶点的法向量，Filters ->Normals,Curvatures and Orientation -> Compute normals for Point sets  （时间长）

5）进行泊松重建，Filters -> Point set -> Surface Reconstruction:Poisson  （八叉树深度调大一些，模型更细致，时间长）

6）点击 Filters -> Selection -> Select faces with edges longer than  （默认参数）

7）右上角中间的delate

8）选择Render -> Lighting -> Double side lighting

9）右侧选中点云模型，选择 Filter ->Sampling -> Vertex Attribute Transfer

10）右侧选中Poisson Mesh，点击 Render -> Color -> Per Vertex

11）进行保存，点击 File -> Export Mesh As,重命名后选择stl文件类型即可

## 9、旋转的在线转换

[https://blog.csdn.net/HW140701/article/details/106255294](https://blog.csdn.net/HW140701/article/details/106255294)

[https://www.andre-gaschler.com/rotationconverter/](https://www.andre-gaschler.com/rotationconverter/)

# 三、badcase问题猜测

## 1、无法到达点的原因猜测：

### （1）是否处于奇点

### （2）手动规定移动路径，存在部分结点跳跃的情况，在实际中是否无法规划运动？？？？

### （3）发出新的碰撞检查请求之前清除碰撞结果???[http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html#change-the-state](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html#change-the-state)

### （4）导入的模型是带机械臂的？？？？？

### （5）偶尔会出现一样的代码，有的时候可以规划到点，有的时候无法规划过去的情况（这种情况本身添加的障碍物的环境比较复杂，无障碍物的情况下一般可以规划过去）

### （6）是否添加了路径规划的约束？？

（是否可以尝试改变求解器：[http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/kinematics_configuration/kinematics_configuration_tutorial.html](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/kinematics_configuration/kinematics_configuration_tutorial.html)）

（是否可以尝试改变规划器的其他方法：[http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ompl_interface/ompl_interface_tutorial.html](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ompl_interface/ompl_interface_tutorial.html)）

修改：/home/longxiaoze/catkin_ws/src/CR5_ROS/cr5_moveit/config/ompl_planning.yaml路径下的default_planner_config: RRT

### （7）替换其他求解器

[https://www.guyuehome.com/1159](https://www.guyuehome.com/1159)

### （8）观察pose为Link6的位姿，不是相机的位姿

修改到相机坐标系之后，仍存在给定badcase无法到达的情况

# 四、其他资料

## 1、添加相机模型到机械臂

#### （1）了解urdf文件

[https://zhuanlan.zhihu.com/p/94701620](https://zhuanlan.zhihu.com/p/94701620)

[https://wiki.ros.org/urdf/Tutorials](https://wiki.ros.org/urdf/Tutorials)

[https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch](https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)

#### （2）将stp转换成dae文件

**使用freecad**

#### （3）添加到urdf文件中

#### （4）点云加载到moveit

[https://blog.csdn.net/HIT_Kyrie/article/details/115901524](https://blog.csdn.net/HIT_Kyrie/article/details/115901524)

[https://www.jianshu.com/p/8ce4cf21a244](https://www.jianshu.com/p/8ce4cf21a244)

## 2、运动规划器

### 针对不同运动规划器和具有规划适配器的规划器的规划见解

[http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_adapters/planning_adapters_tutorial.html#planning-insights-for-different-motion-planners-and-planners-with-planning-adapters](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_adapters/planning_adapters_tutorial.html#planning-insights-for-different-motion-planners-and-planners-with-planning-adapters)

本节深入了解何时使用哪个规划器以及如何在某个管道中使用某些规划请求适配器可以导致总体上产生健壮的路径。在这里，我们考虑单独和一起使用 OMPL、STOMP、CHOMP 来生成从规划器获得的鲁棒平滑优化路径。对于每个规划器，都提供了基本的洞察力，使用户能够直观地在特定情况下使用特定的规划器。

* **CHOMP**：CHOMP 是一种优化算法，可优化给定的初始轨迹。基于环境，CHOMP 迅速尝试将初始轨迹从碰撞中拉出。但是这里需要注意的重要一点是该参数`ridge_factor`需要大于或等于0.001才能避开障碍物。这样做 CHOMP 能够在避开障碍物的同时找到路径。这里应该注意的是，即使 CHOMP 可以成功避开障碍物，但它无法提供平滑的路径，通常会在存在障碍物的情况下导致路径不稳定。对于 CHOMP 来说，避免碰撞是以轨迹的速度平滑度为代价的。
* **STOMP** : STOMP 在合理的时间内产生平滑且性能良好的无碰撞路径。该方法依赖于生成嘈杂的轨迹来探索初始（可能不可行）轨迹周围的空间，然后将其组合以产生成本更低的更新轨迹。
* **OMPL**是一个开源库，用于在 ompl 规划教程中讨论的基于采样/随机运动规划算法。基于采样的算法在概率上是完整的：如果存在解决方案，最终将找到解决方案，但无法报告不存在解决方案。这些算法是高效的，通常可以快速找到解决方案。**（moviet默认使用）**
有关每个运动规划器的更多信息，请参阅它们各自的教程页面[OMPL](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ompl_interface/ompl_interface_tutorial.html)、[CHOMP](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/chomp_planner/chomp_planner_tutorial.html)和[STOMP](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/stomp_planner/stomp_planner_tutorial.html)。 

* 3、python接口文档[moveit_commander](http://docs.ros.org/en/jade/api/moveit_commander/html/namespacemoveit__commander.html)
[http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander-members.html](http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander-members.html)


## 3、bug-error

#### （1）[ERROR] [1644826401.806034065]: Unable to connect to move_group action server 'move_group' within allotted time (30s)

解决：原因-模型过大，加载时间太长

将demo.launch的rviz注释掉，在显示绿色的字符之后，新开一个终端运行rviz

[https://blog.csdn.net/m0_51786202/article/details/121430671](https://blog.csdn.net/m0_51786202/article/details/121430671)

(貌似好像没什么用。。。。。最后还是选择添加box来模拟下方的底盘)


#### （2）cannot get joint stat

注意不要添加中文注释！！！！！会报无法找到joint stat的错误，而且不好debug出来

#### （3）导入stl文件时pyassimp报错

[https://github.com/assimp/assimp/issues/2343](https://github.com/assimp/assimp/issues/2343)

使用4.1.3版本

# 五、进度

|任务|完成情况|备注|
|:----|:----|:----|
|moviet官方教程|OK|官方python的代码运行复现|
|官方教程替换cr5运行|OK|    |
|cr5添加障碍物进行规划|OK|添加box+模拟相机模型|
|cr5的urdf文件中添加相机模型进行规划|OK|    |
|cr5的urdf文件中添加底盘|OK|由于模型太大，导入时间长，后续以box来代替底盘|
|特殊点位badcase无法到达|OK|使用沿着相机方向进行移动    |
|cr5的urdf文件添加障碍物模型|OK|考虑后续工程问题，仅仅作为避障实验使用|
|cr5的python控制文件添加stl模型|OK|后续使用python进行网格文件添加，已完成避障规划。<br>问题：仍然存在网格stl文件加载慢的情况（amdR5700H下需要12s左右）|
|cr5的python控制沿着末端坐标系下，沿着相机朝向方向运动，解决badcase|OK|添加部分的代码约束（移动最大距离搜索为1.5，每次移动0.1，先往z正方向运动，再往反向运动）|
|cr5代码集成，将参数提取到代码最前方，方便后续调参+其他功能入口|OK|（墙壁和mesh文件未关联）|
|将点云ply文件转换为stl网格文件|OK|如需要将pcd文件转化，先转化为ply|
|cr5的python控制文件添加stl模型添加dae底盘文件|OK|    |

## 

# 六、bad_case

## 1、bad_case.yaml

### （1）points

```plain
--------------------------------2-------------------------------------------------------------
  goal:
  position: 
  x: 0.114335
  y: -0.106603
  z: 0.825535
orientation: 
  x: -0.534803
  y: 0.780194
  z: -0.323348
  w: 0.0270134

start:
position: 
  x: 0.283818
  y: 0.257077
  z: 0.743982
orientation: 
  x: 0.878782
  y: -0.350464
  z: 0.18265
```
  w: -0.267499
### （2）说明：

1）



[https://blog.csdn.net/wxflamy/article/details/79179331](https://blog.csdn.net/wxflamy/article/details/79179331)


# 七、添加点云或网格数据到环境中

## 1.导入mesh文件

### （1）dae/stl从urdf文件导入

修改urdf文件（**加载慢，模型太大会报错！！！！**）

### （2）stl从代码导入

#### c++（不成功-自身原因）

参考：MoveIt!环境导入自定义的stl文件

[https://blog.csdn.net/fei_6/article/details/80443171](https://blog.csdn.net/fei_6/article/details/80443171)

关键函数的官方文档：

[https://github.com/hcrmines/apc/blob/master/src/add_objects.cpp](https://github.com/hcrmines/apc/blob/master/src/add_objects.cpp)

[http://wiki.ros.org/geometric_shapes](http://wiki.ros.org/geometric_shapes)

[http://docs.ros.org/en/melodic/api/geometric_shapes/html/namespaceshapes.html#ac8bcf235bea4d4cd524651eee07b0c92](http://docs.ros.org/en/melodic/api/geometric_shapes/html/namespaceshapes.html#ac8bcf235bea4d4cd524651eee07b0c92)

[https://answers.ros.org/question/61094/import-stl-into-moveit/](https://answers.ros.org/question/61094/import-stl-into-moveit/)

[https://github.com/fjonath1/moveit_collision_environment](https://github.com/fjonath1/moveit_collision_environment)

#### python:（使用）

[https://answers.ros.org/question/330092/receiving-meshes-via-ros-in-python/](https://answers.ros.org/question/330092/receiving-meshes-via-ros-in-python/)

[http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html#a6a4d639307ce822c5183ad4368b297e4](http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html#a6a4d639307ce822c5183ad4368b297e4)**(使用的是这个)**

**此方法可以支持的3D格式见如下网站：**[https://assimp-docs.readthedocs.io/en/latest/about/introduction.html](https://assimp-docs.readthedocs.io/en/latest/about/introduction.html)

### （3）从rviz导入

[https://answers.ros.org/question/219584/problem-inserting-stl-or-scene-object-in-rviz/](https://answers.ros.org/question/219584/problem-inserting-stl-or-scene-object-in-rviz/)(不成功)




# 备注：

## 1、需要修改的代码

### （1）机械臂的urdf文件：

路径：~/catkin_ws/src/CR5_ROS/dobot_description/urdf/cr5_robot.urdf

（需要将dae和stl文件放在~/catkin_ws/src/CR5_ROS/dobot_description/meshes/cr5/路径下）

#### 代码：（展开可查看）

```plain
<?xml version="1.0" encoding="utf-8"?>
<!-- This URDF was automatically created by SolidWorks to URDF Exporter! Originally created by Stephen Brawner (brawner@gmail.com) 
     Commit Version: 1.6.0-1-g15f4949  Build Version: 1.6.7594.29634
     For more information, please see http://wiki.ros.org/sw_urdf_exporter -->
<robot name="cr5_robot">
  <link name="dummy_link" />
  <link name="base_link">
    <inertial>
      <origin xyz="-8.0961E-06 0.0019103 0.028995" rpy="0 0 0" />
      <mass value="0.90882" />
      <inertia
        ixx="0.0014545"
        ixy="4.2968E-08"
        ixz="-1.8199E-07"
        iyy="0.001345"
        iyz="-6.243E-06"
        izz="0.002155" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/base_link.dae" />
      </geometry>
      <material
        name="">
        <color rgba="0 0 0 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/base_link.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="dummy_joint" type="fixed">
    <parent link="dummy_link" />
    <child  link="base_link" />
  </joint>

  <link  name="Link1">
    <inertial>
      <origin xyz="-8.7268E-07 -0.0032576 -0.0076748" rpy="0 0 0" />
      <mass value="1.6123" />
      <inertia
        ixx="0.0029122"
        ixy="5.7387E-09"
        ixz="1.9945E-08"
        iyy="0.0028191"
        iyz="0.00044094"
        izz="0.0032836" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link1.dae" />
      </geometry>
      <material
        name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link1.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.147" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="Link1" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint>

  <link name="Link2">
    <inertial>
      <origin xyz="-0.21351 -1.3842E-06 0.14252" rpy="0 0 0" />
      <mass value="5.5186" />
      <inertia
        ixx="0.0085919"
        ixy="7.5552E-07"
        ixz="-7.8922E-07"
        iyy="0.027553"
        iyz="1.4629E-07"
        izz="0.028294" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link2.dae" />
      </geometry>
      <material
        name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link2.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="joint2" type="revolute">
    <origin xyz="0 0 0" rpy="1.5708 1.5708 0" />
    <parent link="Link1" />
    <child  link="Link2" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint>

  <link name="Link3">
    <inertial>
      <origin xyz="-0.15231 -1.4079E-07 0.022693" rpy="0 0 0" />
      <mass value="2.8536" />
      <inertia
        ixx="0.0032608"
        ixy="-3.2147E-10"
        ixz="0.00012363"
        iyy="0.0092967"
        iyz="2.9478E-10"
        izz="0.0095552" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link3.dae" />
      </geometry>
      <material
        name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"  rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link3.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="joint3" type="revolute">
    <origin xyz="-0.427 0 0" rpy="0 0 0" />
    <parent link="Link2" />
    <child link="Link3" />
    <axis  xyz="0 0 1" />
    <limit lower="-3.14"  upper="3.14"  effort="0" velocity="0" />
  </joint>

  <link name="Link4">
    <inertial>
      <origin xyz="-3.2386E-07 -0.002077 -0.0015548" rpy="0 0 0" />
      <mass value="0.67058" />
      <inertia
        ixx="0.00066939"
        ixy="4.1975E-10"
        ixz="-1.0734E-08"
        iyy="0.00065424"
        iyz="0.00011081"
        izz="0.00065365" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link4.dae" />
      </geometry>
      <material name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link4.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="joint4" type="revolute">
    <origin xyz="-0.357 0 0.141" rpy="0 0 -1.5708" />
    <parent link="Link3" />
    <child link="Link4" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint>
  <link
    name="Link5">
    <inertial>
      <origin xyz="-3.7738E-07 -0.0045777 -0.0012971" rpy="0 0 0" />
      <mass value="0.7269" />
      <inertia
        ixx="0.00073069"
        ixy="-5.7806E-09"
        ixz="6.7132E-09"
        iyy="0.00071195"
        iyz="-0.00011926"
        izz="0.00071391" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link5.dae" />
      </geometry>
      <material name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link5.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="joint5" type="revolute">
    <origin xyz="0 -0.116 0" rpy="1.5708 0 0" />
    <parent link="Link4" />
    <child link="Link5" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint>

  <link name="Link6">
    <inertial>
      <origin xyz="5.0777E-11 9.3774E-05 -0.019285" rpy="0 0 0" />
      <mass value="0.15789" />
      <inertia
        ixx="7.8045E-05"
        ixy="6.1582E-15"
        ixz="6.2471E-14"
        iyy="7.9168E-05"
        iyz="-1.3822E-07"
        izz="0.00012951" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link6.dae" />
      </geometry>
      <material name="">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/Link6.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="joint6" type="revolute">
    <origin xyz="0 0.105 0"  rpy="-1.5708 0 0" />
    <parent link="Link5" />
    <child link="Link6" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint>

  <link name="zivid-waike">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/zivid-waike.dae" />
      </geometry>
      <material name="">
        <color rgba="0 0 0 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/zivid-waike.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="zivid" type="fixed">
    <origin xyz="0 0 0.001"  rpy="-1.5708 0 -1.5708" />
    <parent link="Link6" />
    <child link="zivid-waike" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint>
  
  <!-- <link name="BOGIE2">
    <visual>
      <origin xyz="-0.5 1 0.25" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/bogie_del5_real_scale_half.stl" />
      </geometry>
      <material name="">
        <color rgba="0 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="-0.5 1 0.25" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/bogie_del5_real_scale_half.stl" />
      </geometry>
    </collision>
  </link>

  <joint name="bogie2" type="fixed">
    <origin xyz="0 0 -0.001"  rpy="0 0 0" />
    <parent link="base_link" />
    <child link="BOGIE2" />
    <axis xyz="0 0 0" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint> -->

  <!-- <link name="BOGIE1">
    <visual>
      <origin xyz="0 0 2" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/bogie.dae" />
      </geometry>
      <material name="">
        <color rgba="0 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/bogie.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="bogie1" type="fixed">
    <origin xyz="0 0 -0.001"  rpy="0 0 0" />
    <parent link="base_link" />
    <child link="BOGIE1" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint> -->
  
  <!-- use agv can see, but load_time is too long, so we only use box to planning as before-->
  <!-- <link name="AGV">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/AGV.dae" />
      </geometry>
      <material name="">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://dobot_description/meshes/cr5/AGV.dae" />
      </geometry>
    </collision>
  </link>

  <joint name="agv" type="fixed">
    <origin xyz="0 0 -0.001"  rpy="0 0 0" />
    <parent link="base_link" />
    <child link="AGV" />
    <axis xyz="0 0 1" />
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0" />
  </joint> -->
</robot>
```

### （2）cr5的控制python代码：

路径：~/ws_moveit/src/moveit_tutorials/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial_cr5_add_3d_zivid_point_check_load_mesh_tf_new.py

#### 代码：（展开可查看）

```python
#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# import pyassimp
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane
import tf
import math
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
from tf2_ros import TransformException
## END_SUB_TUTORIAL



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    
    robot = moveit_commander.RobotCommander()
    print "debug-=-------------------------"
    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "cr5_arm"
    
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    print "============ Printing robot pose"
    print move_group.get_current_pose()
    print ""

    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.listener = tf.TransformListener()

    # all_pose is [geometry_msgs.Pose().position.x , y ,z , geometry_msgs.Pose().orientation.x, y ,z ,w]
    self.pose_start = [0.313478,0.25734,0.756146,0.882028,-0.350559,0.167868,-0.266376]
    self.pose_goal = [0.149814,-0.108493,0.839925,-0.542926,0.778994,-0.313085,0.0194277]

    # self.go_to_pose_use_tf() args
    self.transformPose_name_end_link = 'Link6'  # can change this to 'zivid-waike' tf_listener_get_pose  listener.transformPose('Link6',pose)
    self.transformPose_name_end_link_begin = 'base_link'  # tf_listener_get_pose  listener.transformPose('base_link',pose_tf_pose)
    
    # self.add_meshes() args
    # self.add_mesh_pose = [-0.5,1.1,0.25,0,0,0,0]
    # self.add_mesh_pose = [0.197784,0.397331,-0.383783,0.0894667,-0.28682,0.710938,0.635844]
    # self.add_mesh_pose = [0,0,0,0,0,0,0]
    # self.add_mesh_pose = [0.5389,-0.4035,0.8156,-0.1669,-0.3911,-0.3456,0.8364]  #POSE1 ERROR
    # self.add_mesh_pose = [0.1211,-0.0500,0.4761,-0.0716,0.4138,0.8276,0.3723]    #POSE2 ERROR
    self.add_mesh_pose = [0.151587,0.145192,-0.220096,0.0494624,-0.158571,0.393048,0.90439]    #POSE20220225--right!!!-->'/home/longxiaoze/3D_models/stop/Mesh[unpower_bogie_meter_1_mesh.stl'
    # self.file_stl_name = '/home/longxiaoze/3D_models/bogie_del5_real_scale_half.stl'
    # self.file_stl_name = '/home/longxiaoze/3D_models/unpower_bogie_meter.ply'   #this format can load
    # self.file_stl_name = '/home/longxiaoze/3D_models/bogie_scale.stl'           #this format can load
    # self.file_stl_name = '/home/longxiaoze/3D_models/AGV.dae'                   #this format can load
    # self.file_stl_name = '/home/longxiaoze/3D_models/try2trans.stl'
    self.file_stl_name = '/home/longxiaoze/3D_models/stop/Mesh[unpower_bogie_meter_1_mesh.stl'
    
    # self.add_agv_meshes() args
    self.add_agv_pose = [0,0,0,0, 0, 0.3894183, 0.921061]
    self.file_agv_name = '/home/longxiaoze/3D_models/AGV.dae'

    # self.add_wall() args  TODO:need to change this and link to mesh!!!!!!!
    self.add_wall_left_pose = [-0.59,0.06,0,0.0010418, -0.0004277, 0.3855434, -0.922689]################################################
    self.add_wall_right_pose = [-0.06,0-0.75,0,0.0324761, -0.0139378, -0.371269, 0.9278526]################################################
    self.add_wall_size = [0.1, 2, 2]

    # self.add_ground() args
    self.add_ground_pose = [0,0,-0.1,0,0,0,0]



  def tf_listener_get_pose(self,pose_goal,distance = 0.2):
    listener = self.listener
    rate = rospy.Rate(10.0)
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'

    pose.pose.position.x = pose_goal.position.x
    pose.pose.position.y = pose_goal.position.y
    pose.pose.position.z = pose_goal.position.z
    pose.pose.orientation.x =  pose_goal.orientation.x
    pose.pose.orientation.y = pose_goal.orientation.y
    pose.pose.orientation.z = pose_goal.orientation.z
    pose.pose.orientation.w = pose_goal.orientation.w

    print pose
    try:
      print '++++++++++++++++++++++++++++++++++++'
      # pose_tf = listener.transformPose('zivid-waike',pose)
      pose_tf = listener.transformPose(self.transformPose_name_end_link , pose)
      print pose_tf
      print '++++++++++++++++++++++++++++++++++++'
      pose_tf_pose = pose_tf
      pose_tf_pose.pose.position.z += distance
      pose_tf_pose = listener.transformPose(self.transformPose_name_end_link_begin,pose_tf_pose)
      print pose_tf_pose
      print '++++++++++++++++++++++++++++++++++++'
      return pose_tf_pose

    except :
      return None

  def go_to_pose_use_tf(self):   
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.266376
    # pose_goal.position.x = 2.95976602748
    # pose_goal.position.y = 3.31944757968
    # pose_goal.position.z = 3.69224151038
    # pose_goal.orientation.x =  0.882028
    # pose_goal.orientation.y = -0.350559
    # pose_goal.orientation.z = 0.167868

    # #(1)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0194277
    # pose_goal.position.x = 0.149814
    # pose_goal.position.y = -0.108493
    # pose_goal.position.z = 0.839925
    # pose_goal.orientation.x =  -0.542926
    # pose_goal.orientation.y = 0.778994
    # pose_goal.orientation.z = -0.313085

    #(2)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.0270134
    # pose_goal.position.x =  0.114335
    # pose_goal.position.y = -0.106603
    # pose_goal.position.z = 0.825535
    # pose_goal.orientation.x =  0.534803
    # pose_goal.orientation.y = -0.780194
    # pose_goal.orientation.z = 0.323348

    # #(3)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.22105
    # pose_goal.position.x = -0.304324
    # pose_goal.position.y = -0.360256
    # pose_goal.position.z = 0.786937
    # pose_goal.orientation.x =  -0.117548
    # pose_goal.orientation.y = 0.944906
    # pose_goal.orientation.z = -0.210876
    # #(4)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.224127
    # pose_goal.position.x = -0.186267
    # pose_goal.position.y = -0.460263
    # pose_goal.position.z = 0.788241
    # pose_goal.orientation.x =  -0.157511
    # pose_goal.orientation.y = 0.942456
    # pose_goal.orientation.z = -0.191661
    # #(5)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0506901
    # pose_goal.position.x = 0.139795
    # pose_goal.position.y = -0.0515554
    # pose_goal.position.z = 0.827831
    # pose_goal.orientation.x =  -0.587747
    # pose_goal.orientation.y =  0.743891
    # pose_goal.orientation.z = -0.314022

    #(2)
    pose_goal_set = self.pose_goal
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose_goal_set[0]
    pose_goal.position.y = pose_goal_set[1]
    pose_goal.position.z = pose_goal_set[2]
    pose_goal.orientation.x =  pose_goal_set[3]
    pose_goal.orientation.y = pose_goal_set[4]
    pose_goal.orientation.z = pose_goal_set[5]
    pose_goal.orientation.w = pose_goal_set[6]

    flag = True
    try_num = 15
    count_num = try_num
    distance = 0.1
    while flag == True:
      print distance
      pose_tf_pose = self.tf_listener_get_pose(pose_goal,distance)

      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.position.x = pose_tf_pose.pose.position.x
      pose_goal.position.y = pose_tf_pose.pose.position.y
      pose_goal.position.z = pose_tf_pose.pose.position.z
      pose_goal.orientation.x =  pose_tf_pose.pose.orientation.x
      pose_goal.orientation.y = pose_tf_pose.pose.orientation.y 
      pose_goal.orientation.z = pose_tf_pose.pose.orientation.z 
      pose_goal.orientation.w = pose_tf_pose.pose.orientation.w

      # move_group.set_pose_target(pose_goal,end_effector_link ="zivid-waike")
      move_group.set_pose_target(pose_goal)

      
      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group.go(wait=True)
      print plan
      # Calling `stop()` ensures that there is no residual movement
      move_group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      move_group.clear_pose_targets()
      if plan ==False:
        print 'plan_error plan again, now we delate on last link in z axis with same orientation and plan again!'
        try_num  = try_num - 1
        print try_num
        if try_num == 0:
          distance = -0.1-count_num*0.1
        elif try_num == -1:
          distance = -0.1
        if try_num == -15:
          print "move cannot find a plan, this pose can not use!!!!!!!!!!!!!!!!!!"
          break
      else:
        break
        # if try_num == -15:  # keep moving
        #   break             # keep moving
    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def add_agv_meshes(self, timeout=4):
    scene = self.scene
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header.frame_id = "base_link"

    mesh_pose.pose.position.x = self.add_agv_pose[0]
    mesh_pose.pose.position.y = self.add_agv_pose[1]
    mesh_pose.pose.position.z = self.add_agv_pose[2]
    mesh_pose.pose.orientation.x = self.add_agv_pose[3]
    mesh_pose.pose.orientation.y = self.add_agv_pose[4]
    mesh_pose.pose.orientation.z = self.add_agv_pose[5]
    mesh_pose.pose.orientation.w = self.add_agv_pose[6]
    file_stl = self.file_agv_name

    scene.add_mesh('mesh_agv', mesh_pose, file_stl)

  def add_meshes(self, timeout=4):
    scene = self.scene
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header.frame_id = "base_link"
    # # mesh_pose.pose.orientation.w = 1.0
    # # mesh_pose.pose.orientation.x = 1.0
    # # mesh_pose.pose.orientation.y = 1.0
    # # mesh_pose.pose.orientation.z = 1.0
    # mesh_pose.pose.position.x = -0.5 
    # mesh_pose.pose.position.y = 1.1
    # mesh_pose.pose.position.z = 0.25

    mesh_pose.pose.position.x = self.add_mesh_pose[0]
    mesh_pose.pose.position.y = self.add_mesh_pose[1]
    mesh_pose.pose.position.z = self.add_mesh_pose[2]
    mesh_pose.pose.orientation.x = self.add_mesh_pose[3]
    mesh_pose.pose.orientation.y = self.add_mesh_pose[4]
    mesh_pose.pose.orientation.z = self.add_mesh_pose[5]
    mesh_pose.pose.orientation.w = self.add_mesh_pose[6]
    file_stl = self.file_stl_name

    scene.add_mesh('mesh', mesh_pose, file_stl)

  def add_wall(self,wall_name, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene
    if wall_name == 'wall_left':
      wall_pose_set = self.add_wall_left_pose
    elif wall_name =='wall_right':
      wall_pose_set = self.add_wall_right_pose
    else:
      print 'change wall_name to wall_left or wall_right,and set self.add_wall_left_pose etl'
    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    wall_pose = geometry_msgs.msg.PoseStamped()
    wall_pose.header.frame_id = "base_link"
    wall_pose.pose.orientation.x = wall_pose_set[3]
    wall_pose.pose.orientation.y = wall_pose_set[4]
    wall_pose.pose.orientation.z = wall_pose_set[5]
    wall_pose.pose.orientation.w = wall_pose_set[6]
    wall_pose.pose.position.x = wall_pose_set[0] # slightly above the end effector
    wall_pose.pose.position.y = wall_pose_set[1]
    wall_pose.pose.position.z = wall_pose_set[2]
    wall_name = wall_name
    scene.add_box(wall_name, wall_pose, size=(self.add_wall_size[0], self.add_wall_size[1], self.add_wall_size[2]))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def go_to_pose_start(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # # (1)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.266376
    # pose_goal.position.x = 0.313478
    # pose_goal.position.y = 0.25734
    # pose_goal.position.z = 0.756146
    # pose_goal.orientation.x =  0.882028
    # pose_goal.orientation.y = -0.350559
    # pose_goal.orientation.z = 0.167868
    

    #(1)pingyi z = z+0.2
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = 0.409851050058
    # pose_goal.position.y = 0.327822450347
    # pose_goal.position.z = 0.595695199392
    # pose_goal.orientation.x =  0.882027627756
    # pose_goal.orientation.y = -0.350558852053
    # pose_goal.orientation.z = 0.167867929154 
    # pose_goal.orientation.w = -0.266375887581
    # (1)goal-random orientation
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0194277
    # pose_goal.position.x = 0.149814
    # pose_goal.position.y = -0.108493
    # pose_goal.position.z = 0.839925
    # pose_goal.orientation.x =  0.12918
    # pose_goal.orientation.y = 0.0019221
    # pose_goal.orientation.z = 0.95653
    # pose_goal.orientation.w = -0.037766
    # (2)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.267499
    # pose_goal.position.x =  0.283818
    # pose_goal.position.y = 0.257077
    # pose_goal.position.z = 0.743982   
    # pose_goal.orientation.x =  0.878782
    # pose_goal.orientation.y = -0.350464
    # pose_goal.orientation.z = 0.18265
    # # (3)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.0273244
    # pose_goal.position.x =  -0.0420086
    # pose_goal.position.y = -0.212872
    # pose_goal.position.z = 0.808511   
    # pose_goal.orientation.x =  -0.382124
    # pose_goal.orientation.y =  0.862073
    # pose_goal.orientation.z = -0.331761
    # # (4)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.0157909
    # pose_goal.position.x = 0.0627556
    # pose_goal.position.y = -0.291287
    # pose_goal.position.z = 0.809011  
    # pose_goal.orientation.x =  -0.42142
    # pose_goal.orientation.y = -0.321937
    # pose_goal.orientation.z = -0.0157909
    # # (5)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.284415
    # pose_goal.position.x = 0.257385
    # pose_goal.position.y = 0.332951
    # pose_goal.position.z = 0.750592 
    # pose_goal.orientation.x =  0.899413
    # pose_goal.orientation.y = -0.291265
    # pose_goal.orientation.z = 0.159151
    # test_mesh_pose
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x= -0.43813764921
    # pose_goal.position.y= -0.285925896352
    # pose_goal.position.z= 0.820336658446
    # pose_goal.orientation.x = -5.86557414841e-05
    # pose_goal.orientation.y = -0.182421875408
    # pose_goal.orientation.z = -0.983220347671
    # pose_goal.orientation.w = 6.21126577036e-05

    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = self.pose_start[0]
    pose_goal.position.y = self.pose_start[1]
    pose_goal.position.z = self.pose_start[2]
    pose_goal.orientation.x =  self.pose_start[3]
    pose_goal.orientation.y = self.pose_start[4]
    pose_goal.orientation.z = self.pose_start[5]
    pose_goal.orientation.w = self.pose_start[6]

    # move_group.set_pose_target(pose_goal,end_effector_link ="zivid-waike")
    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # #(1)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0194277
    # pose_goal.position.x = 0.149814
    # pose_goal.position.y = -0.108493
    # pose_goal.position.z = 0.839925
    # pose_goal.orientation.x =  -0.542926
    # pose_goal.orientation.y = 0.778994
    # pose_goal.orientation.z = -0.313085

    #(2)
    # pose_goal = geometry_msgs.msg.Pose()
    # # pose_goal.orientation.w = -0.0270134
    # pose_goal.position.x =  0.114335
    # pose_goal.position.y = -0.106603
    # # pose_goal.position.y = -0.18
    # pose_goal.position.z = 0.825535
    # # pose_goal.orientation.x =  0.534803
    # # pose_goal.orientation.y = -0.780194
    # # pose_goal.orientation.z = 0.323348
    # #(3)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.22105
    # pose_goal.position.x = -0.304324
    # pose_goal.position.y = -0.360256
    # pose_goal.position.z = 0.786937
    # pose_goal.orientation.x =  -0.117548
    # pose_goal.orientation.y = 0.944906
    # pose_goal.orientation.z = -0.210876
    # #(4)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.224127
    # pose_goal.position.x = -0.186267
    # pose_goal.position.y = -0.460263
    # pose_goal.position.z = 0.788241
    # pose_goal.orientation.x =  -0.157511
    # pose_goal.orientation.y = 0.942456
    # pose_goal.orientation.z = -0.191661
    # #(5)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0506901
    # pose_goal.position.x = 0.139795
    # pose_goal.position.y = -0.0515554
    # pose_goal.position.z = 0.827831
    # pose_goal.orientation.x =  -0.587747
    # pose_goal.orientation.y =  0.743891
    # pose_goal.orientation.z = -0.314022
    
    # test_mesh_pose
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x= 2.82837076328e-06
    # pose_goal.position.y= -0.245999999999
    # pose_goal.position.z= 1.04700038568
    # pose_goal.orientation.x = 1.29867411873e-06
    # pose_goal.orientation.y = -0.707106781181
    # pose_goal.orientation.z = 0.707106781181
    # pose_goal.orientation.w = 3.89602235603e-06

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = self.pose_goal[0]
    pose_goal.position.y = self.pose_goal[1]
    pose_goal.position.z = self.pose_goal[2]
    pose_goal.orientation.x =  self.pose_goal[3]
    pose_goal.orientation.y = self.pose_goal[4]
    pose_goal.orientation.z = self.pose_goal[5]
    pose_goal.orientation.w = self.pose_goal[6]

    # move_group.set_pose_target(pose_goal,end_effector_link ="zivid-waike")
    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def add_ground(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = self.add_ground_pose[6]
    box_pose.pose.position.z = self.add_ground_pose[2] # slightly above the end effector
    box_name = "ground"
    scene.add_box(box_name, box_pose, size=(1, 1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

########################### not use code #################################################################################
  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step 1cm
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Link6"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.1 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.2))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'cr5_arm' #you should change this name to your end effector group name!!!!!!!!!!!!!!
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def add_area(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Link1"
    box_pose.pose.orientation.w = 0
    box_pose.pose.position.z = 1.38 # slightly above the end effector
    box_pose.pose.position.x = 0.45 # slightly above the end effector
    box_pose.pose.position.y = 0.3 # slightly above the end effector
    box_name = "area"
    scene.add_box(box_name, box_pose, size=(0.1, 2, 2))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.8892652952806639
    joint_goal[1] = 0.7779352074689527
    joint_goal[2] = -1.4838353281846606
    joint_goal[3] = -0.8655974713766813
    joint_goal[4] = -1.5705821053379334
    joint_goal[5] = -2.459579312371455
    #[0.8892652952806639, 0.7779352074689527, -1.4838353281846606,
    # -0.8655974713766813, -1.5705821053379334, -2.459579312371455]
    # joint_goal[6] = 0  #cr5 dont need this

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
########################### not use code #################################################################################

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to add left and right wall to the planning scene ..."
    raw_input()
    tutorial.add_wall("wall_left")
    tutorial.add_wall("wall_right")

    print "============ Press `Enter` to add_mesh================== ..."
    raw_input()
    tutorial.add_meshes()

    print "============ Press `Enter` to add_agv_mesh================== ..."
    raw_input()
    tutorial.add_agv_meshes()

    # print "============ Press `Enter` to add a ground to the planning scene ..."
    # raw_input()
    # tutorial.add_ground()

    print "============ Press `Enter` to execute a movement using a start goal ..."
    raw_input()
    tutorial.go_to_pose_start()
    print "============ Press `Enter` to execute a movement using go_to_pose_use_tf ..."
    raw_input()
    tutorial.go_to_pose_use_tf()









    # print "============ Press `Enter` to add a area to the planning scene ..."
    # raw_input()
    # tutorial.add_area()

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    # tutorial.go_to_joint_state()

    # print "============ Press `Enter` to execute a movement using a pose goal ..."
    # raw_input()
    # tutorial.go_to_pose_goal()

    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to detach the box from the Panda robot ..."
    # raw_input()
    # tutorial.detach_box()

    # print "============ Press `Enter` to remove the box from the planning scene ..."
    # raw_input()
    # tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
```


