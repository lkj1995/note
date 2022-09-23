- PFD的根时钟来源于PLL

- PLL2（system PLL）推荐频率为528MHz，用于提供USB模块或4路PFD输出

- PFD可根据实际情况配置成不同频率的时钟源，用于为其他模块提供时钟源，通过CCM来选择其路径

- 由于PLL2支持扩频调制功能（用于减少辐射干扰），因此**锁定时间**比不支持扩频调制功能的PLL更长

- PLL2生成后，其PFD可以大于PLL或小于PLL，取决于公式 Fpfd = F * 18 / N 。

- PLL3（usb1 PLL）固定频率为480MHz，用于提供USB模块或4路PFD输出

- 可直接改变PFD频率，而无需重新锁定ROOT PLL

- PFD因是数字设计部件，无模拟部件，所以切换频率的速度比PLL会更快，此特性用于dynamic voltage and frequency scaling（DVFS）