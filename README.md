# 灵巧手使用说明

## 硬件

### 接口与线序

由于因时灵巧手长度过长，部分灵巧手存在拆除下半部分的情况，以下简称为“手腕”。未拆掉手腕的灵巧手线序与接口如图所示：

<img src="images/Screenshot 2024-03-06 at 19.25.02.png" alt="Screenshot 2024-03-06 at 19.25.02" style="zoom:50%;" />

注意：

- 通过手腕接口接入灵巧手的VCC输入为24V，经过一个降压板后转为7-9V，降压板如图所示：

<img src="images/image-20240306192330229.png" alt="image-20240306192330229" style="zoom:20%;" />

- 用于双臂倒茶demo的灵巧手为RS232的通信方式

拆掉手腕后，灵巧手的线序如下图和下表所示：

<img src="images/image-20240306192444281.png" alt="image-20240306192444281" style="zoom:20%;" />

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

### ROS Package

使用以下命令克隆因时灵巧手的ros package并用你喜欢的编译命令对其进行编译：

```bash
# git clone 
git clone https://github.com/MickyFlowers/inspire_hand.git
# build
catkin build
```

运行灵巧手的文件为`hand_control.launch`，文件内容如下所示：

```yaml
<?xml version="1.0" ?>
<launch>
  <arg name="port" default= "/dev/ttyS1" />
  <arg name="baud" default= "115200" />
  <arg name="test_flag" default= "0" />
  <node name="inspire_hand" pkg="inspire_hand" type="inspire_hand" output="screen" >
    <param name = "portname" value="$(arg port)" />
    <param name = "baudrate" value="$(arg baud)" />
    <param name = "test_flags" value="$(arg test_flag)" />
  </node>
</launch>
```

该文件中运行了灵巧手控制节点，在使用时候需要更改文件内容，将灵巧手端口号和波特率进行对应，而后运行命令：

```bash
roslaunch inspire_hand hand_control.launch [args]
```

该节点以ROS Service Server的形式运行，可以在命令行中查看所有可调用的服务：

```bash
rosservice list
```

以读写右手灵巧手关节数据为例：

```bash
# 读取
rosservice call /inspire_hand/get_angle_act "{id: 0}"
# 写(angle1-6替换为0-1000的数即可改变灵巧手关节角度值)
rosservice call /right_inspire_hand/set_angle "{id: 0, angle1: 0, angle2: 0, angle3: 0, angle4: 0, angle5: 0, angle6: 0}"
```

