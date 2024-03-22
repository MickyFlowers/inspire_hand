# 灵巧手使用说明

## 硬件

### 接口与线序

由于因时灵巧手长度过长，部分灵巧手存在拆除下半部分的情况，以下简称为“手腕”。未拆掉手腕的灵巧手线序与接口如图所示：
<div align=center>
<img src="images/Screenshot 2024-03-06 at 19.25.02.png" alt="Screenshot 2024-03-06 at 19.25.02" width="500px" />
</div>
注意：

- 通过手腕接口接入灵巧手的VCC输入为24V，经过一个降压板后转为7-9V，降压板如图所示：
<div align=center>
<img src="images/image-20240306192330229.png" alt="image-20240306192330229" width="200px" />
</div>
- 用于双臂倒茶demo的灵巧手为RS232的通信方式

拆掉手腕后，灵巧手的线序如下图和下表所示：
如图所示：
<div align=center>
<img src="images/image-20240306192444281.png" alt="image-20240306192444281" width="200px" />
</div>
| 线路颜色 |  引脚定义   |
| :------: | :---------: |
|    黑    |     GND     |
|    红    | VCC（7-9V） |
|    黄    |  RS485 A+   |
|    蓝    |  RS485 B-   |

部分灵巧手可能存在颜色不同的情况，可参考下表：

| 线路颜色 |  引脚定义   |
| :------: | :---------: |
|    黑    |     GND     |
|    红    | VCC（7-9V） |
|    白    |  RS485 A+   |
|    灰    |  RS485 B-   |

## 控制灵巧手

### 上位机（Windows）

上位机与上位机操作说明下载网址：https://www.inspire-robots.com/download/frwz/

注意：

- 在确保线路无问题的情况下，可能会出现无法连上的情况，此时可以尝试重新插拔灵巧手的电源和串口。
- 部分灵巧手可能存在修改波特率的情况，在连接时请尝试多种波特率。

### 快速开始
在代码中实例化一个inspire_hand::InspireHandSerial对象，传入端口和波特率，即可初始化inspire_hand的端口
```c++
inspire_hand::InspireHandSerial inspire_hand_serial("/dev/ttyUSB0", 115200);
```
该类包含以下方法可供调用

### ROS Package

使用以下命令克隆因时灵巧手的ros package并用你喜欢的编译命令对其进行编译：

```bash
# git clone 
git clone https://github.com/MickyFlowers/inspire_hand.git -b fd_version
# build
catkin build
```

运行灵巧手的文件为`inspire_hand_server.launch`，文件内容如下所示：

```yaml
<?xml version="1.0" ?>
<launch>
  <arg name="port" default= "/dev/ttyUSB0" />
  <arg name="baudrate" default= "115200" />
  <node name="inspire_hand_server" pkg="inspire_hand" type="inspire_hand_server" output="screen" >
    <param name = "port" value="$(arg port)" />
    <param name = "baudrate" value="$(arg baudrate)" />
  </node>
</launch>
```

该launch文件启动了端口并能够控制总线上所有的灵巧手运行命令：

```bash
roslaunch inspire_hand inspire_hand_server.launch port:="/dev/ttyS1" baudrate:=115200
```

该节点以ROS Service Server的形式运行，可以在命令行中查看所有可调用的服务：

```bash
rosservice list
```

以写id=1的灵巧手关节数据为例：

```bash

# 写(angle1-6替换为0-1000的数即可改变灵巧手关节角度值)
rosservice call /right_inspire_hand/set_angle id angle1 angle2 angle3 angle4 angle5 angle6
```

