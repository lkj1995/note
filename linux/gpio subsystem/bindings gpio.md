- 在节点中，gpio属性应该被命名为 " [<name>-] gpios"，name字段用于区分是哪个device的gpio。
- gpio区分为逻辑电平和物理电平，这样可达到软件抽象上的一致性，如led的开和关，可以是低电平有效（开启），可以是高电平有效（开启）。

- ngpios代表的意思是，gpio寄存器的可使用slot数量，即寄存器是32位宽，但只有18位对应了控制引脚，1个gpio-controller控制18个pin，设置"ngpios = <18>，通知driver只有18个pin。

