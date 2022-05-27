## properties标准

### compatible

- 属性名：compatible

- 值类型：stringlist

- 描述：包含1个或多个字符串

- 格式：“厂商,型号”

- 示例：

  -  compatible = "fsl,mpc8641", "ns16550";  
  - 寻找符合第一个字符串的driver，如失败，则会匹配第二个。

  

### model 

- 属性名：model

- 值类型：string

- 描述：
- 格式：“厂商,型号”

- 示例：
  - model = "fsl,MPC8349EMITX";  

### phandle 

- 属性名：phandle 

- 值类型：u32

- 描述：唯一标识符，用于在设备树来识别该node.一般不需显式出现，dtc会自动插入该属性
- 格式：phandle = <num>;

- 示例：
  - phandle = <1>; 

### status 

- 属性名：status 

- 值类型：string

| 类型       | 描述                                                         |
| ---------- | ------------------------------------------------------------ |
| "okay"     | 该设备可操作。                                               |
| "disabled" | 该设备当前不可操作，但可能在之后能够操作，如热拔插。         |
| "reserved" | 该设备可操作，但不应该被使用，被其他软件部件控制。           |
| "fail"     | 说明该设备不可操作，一个错误码被记录在设备中，被修复前都无法再操作。 |
| "fail-sss" | 同上，sss为错误码。                                          |
- 格式：status = <string>;

- 示例：
  - status= “okay“