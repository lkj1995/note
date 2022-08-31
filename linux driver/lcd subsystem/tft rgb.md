#### 1 SYNC 时序
![](E:\gitnote\linux driver\lcd subsystem\img\timer.jpg)

- DOTCLK：像素点时钟，每产生一个时钟，电子枪打一个点到屏幕上。
- HSYNC：行同步信号，当横向打完一行的点后，产生一个信号，电子枪则会切换到下一行。

- HSYNC Pulse Width：行同步信号有效宽度。
- VFP：右边的无效点个数。屏幕存在有效区域，当打完一行的点后，还会继续往前打一些无效的点。

- VBP：左边的无效点个数。
- VSYNC：列同步信号，当整个屏幕打完点后，产生一个信号，电子枪会从右下角切换到左上角。

- VSYNC Pulse Width：列同步信号有效宽度。
