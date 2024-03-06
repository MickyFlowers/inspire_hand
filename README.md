# 灵巧手使用说明

## 硬件

### 接口与线序

由于因时灵巧手长度过长，部分灵巧手存在拆除下半部分的情况，以下简称为“手腕”。未拆掉手腕的灵巧手线序与接口如图所示：

<img src="/Users/mac/Library/Application Support/typora-user-images/Screenshot 2024-03-06 at 16.11.58.png" alt="Screenshot 2024-03-06 at 16.11.58" style="zoom:50%;" />

注意：

- 通过手腕接口接入灵巧手的VCC输入为24V，经过一个降压板后转为7-9V，降压板如图所示：

<img src="/Users/mac/Library/Application Support/typora-user-images/image-20240306161621243.png" alt="image-20240306161621243" style="zoom:20%;" />

- 用于双臂倒茶demo的灵巧手为RS232的通信方式

拆掉手腕后，灵巧手的线序如下图和下表所示：

<img src="/Users/mac/Library/Application Support/typora-user-images/image-20240306161704946.png" alt="image-20240306161704946" style="zoom:20%;" />

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
git clone 
```

