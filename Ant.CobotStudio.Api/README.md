本项目是以通用抓取平台的模型作为基础，添加基础的机器人运动控制代码。

`sudo vim /etc/udev/rules.d/70-ttyusb.rules`

`增加如下内容：`

`KERNEL=="ttyUSB[0-9]*",MODE="0666"`

`保存，重新插入USB转串口，普通用户就能搞定了`