#### 什么是ramfs
 - ramfs无备份功能，当掉电后ramfs保存的文件会丢失。写入文件时，按照缓存机制申请page或dentries，但其pages会被锁定，无法进行vm子系统的换页回收，在文件被移除后才能回收。

- 实现ramfs所需的代码很少，几乎不占空间，因为它的功能是基于系统的disk缓存机制。ramfs的实现就是将一部分disk cache当作文件系统挂载。因此ramfs是个可选的组件，可以在kernel的menuconfig开启或关闭。

- tmpfs是ramfs的派生物，继承ramfs优点的同时，也解决了其缺点。
