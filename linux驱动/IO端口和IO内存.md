# IO端口和IO内存

## IO端口部分

1. IO端口基本用在x86架构的芯片，内存和外设寄存器的地址是分离的，操作外设寄存器通过特殊指令。

2. 使用IO端口之前，需要进行注册，使用完毕后，需要释放。

   ```c
   /**
    * __request_region - create a new busy resource region
    * @parent: parent resource descriptor
    * @start: resource start address
    * @n: resource region size
    * @name: reserving caller's ID string
    * @flags: IO resource flags
    */
   struct resource * __request_region(struct resource *parent, 
                                      resource_size_t start, 
                                      resource_size_t n, 
                                      const char *name, 
                                      int flags);
   
   
   /**
    * __request_region - create a new busy resource region
    * @parent: parent resource descriptor
    * @start: resource start address
    * @n: resource region size
    * @name: reserving caller's ID string
    * @flags: IO resource flags
    */
   struct resource * __request_region(struct resource *parent,
                                      resource_size_t start, 
                                      resource_size_t n,
                                      const char *name, 
                                      int flags);
   ```

3. 操作IO端口

   ```c
   u8 inb(unsigned long port)
   {
   	return ioread8(ioport_map(port, 1));
   }
   
   u16 inw(unsigned long port)
   {
   	return ioread16(ioport_map(port, 2));
   }
   
   u32 inl(unsigned long port)
   {
   	return ioread32(ioport_map(port, 4));
   }
   
   void outb(u8 b, unsigned long port)
   {
   	iowrite8(b, ioport_map(port, 1));
   }
   
   void outw(u16 b, unsigned long port)
   {
   	iowrite16(b, ioport_map(port, 2));
   }
   
   void outl(u32 b, unsigned long port)
   {
   	iowrite32(b, ioport_map(port, 4));
   }
   
   ```
   
4. 串操作，传输字符串，使用一条机器指令实现，或是紧凑循环实现。

   ```c
   void insb(unsigned port, void *addr, unsigned long count);
   ...
   ```

5. 暂停式IO，用于x86架构的处理器，处理速度过快和南桥北桥速度匹配不上的问题。

   

## IO内存部分

1. IO内存用在RAM架构，外设寄存器和内存共用地址总线，分区域。

2. 需要将外设寄存器实际物理地址映射到虚拟地址。

   ```C
   void __iomem *ioremap(phys_addr_t paddr, unsigned long size); //映射
   void iounmap(const void __iomem *addr);  //取消映射
   ```

3. request_mem_region仅仅是linux对IO内存的管理，意思指这块内存我已经占用了，别人就不要动了，也不能被swap出去。使用这些寄存器时，可以不调用request_mem_region，但这样的话就不能阻止别人对他的访问了。

   ```c
   #define request_region(start,n,name) __request_region(&ioport_resource, (start), (n), (name), 0)
   
   #define request_muxed_region(start,n,name) __request_region(&ioport_resource, (start), (n), (name), IORESOURCE_MUXED)
   
   #define __request_mem_region(start,n,name, excl) __request_region(&iomem_resource, (start), (n), (name), excl)
   
   #define request_mem_region(start,n,name) __request_region(&iomem_resource, (start), (n), (name), 0)

4. 使用IO内存

   ```C
   /*单个寄存器读写*/
   unsigned int ioread8(const void __iomem *addr)
   {
   	unsigned int ret;
   	mb();
   	ret = IO_CONCAT(__IO_PREFIX,ioread8)(addr);
   	mb();
   	return ret;
   }
   
   unsigned int ioread16(const void __iomem *addr)
   {
   	unsigned int ret;
   	mb();
   	ret = IO_CONCAT(__IO_PREFIX,ioread16)(addr);
   	mb();
   	return ret;
   }
   
   unsigned int ioread32(const void __iomem *addr)
   {
   	unsigned int ret;
   	mb();
   	ret = IO_CONCAT(__IO_PREFIX,ioread32)(addr);
   	mb();
   	return ret;
   }
   
   void iowrite8(u8 b, void __iomem *addr)
   {
   	mb();
   	IO_CONCAT(__IO_PREFIX,iowrite8)(b, addr);
   }
   
   void iowrite16(u16 b, void __iomem *addr)
   {
   	mb();
   	IO_CONCAT(__IO_PREFIX,iowrite16)(b, addr);
   }
   
   void iowrite32(u32 b, void __iomem *addr)
   {
   	mb();
   	IO_CONCAT(__IO_PREFIX,iowrite32)(b, addr);
   }
   ```

   ```c
    /* 内存块读写*/
   
   /*
    * Copy data from IO memory space to "real" memory space.
    * This needs to be optimized.
    */
   void memcpy_fromio(void *to, const volatile void __iomem *from, long count);
   
   /*
    * Copy data from "real" memory space to IO memory space.
    * This needs to be optimized.
    */
   void memcpy_toio(volatile void __iomem *to, const void *from, long count);
   ```

   




## 其他部分

1. 编译器会对代码进行优化处理。或者是高速缓存的读写，如果数据未命中则会从主存中查找，再次未命中则会去磁盘查找。因此可能会存在优化后的代码存在顺序不是按照编写的顺序执行，因此驱动程序要确保不使用高速缓存，并且要对此部分防止优化。按顺序执行，每次读写操作都要重新执行（volatile）。

2. 插入内存屏障的函数防止编译器优化。

   ```c
   void rmb(void); //保证屏障之前的读操作一定会在后来的读操作之前完成
   void wmb(void); //保证屏障之前的写操作一定会在后来的写操作之前完成
   void mb(void);  //两者都不会
   ```

3. 用于寄存器初始化操作，因为初始化当前芯片外设功能或外部的芯片，需要严格按照顺序执行。

