#### 1 下载tslib

```shell
mkdir ~/my_tslib

cd my_tslib

git clone https://github.com/libts/tslib.git
```

#### 2 配置环境变量

```shell
export $ARCH=arm
export $CROSS_COMPILE=arm-bulidroot-linux-gnueabihf
```

#### 3 安装工具

```shell
sudo apt install autogen autoconf libtool
```

#### 4 打开configure脚本

```shell
./configure --host=arm-bulidroot-linux-gnueabihf  --prefix=/
```

#### 5 make编译

```shell
make
```

#### 6 生成lib，inc，bin，etc文件的位置

```shell
make install DESTDIR=${PWD}/tmp
```

#### 7 复制文件到工具链编译的路径

```shell
cd ~/my_tslib/tslib/tmp

cp ./include/*.h /home/book/100ask_imx6ull-sdk/ToolChain/arm-buildroot-linux-gnueabihf_sdk-buildroot/arm-buildroot-linux-gnueabihf/sysroot/usr/include

cp ./include/*.so* /home/book/100ask_imx6ull-sdk/ToolChain/arm-buildroot-linux-gnueabihf_sdk-buildroot/arm-buildroot-linux-gnueabihf/sysroot/usr/lib

这时使用 #include <tslib.h>，就可以调用tslib的函数
```

#### 8 挂载文件nfs

```shell
mount -t nfs -o nolock,vers=3 192.168.0.136:/home/book /mnt
```

#### 9 复制文件

```shell
cd /mnt/my_tslib/tslib/tmp

cp lib/*so* /lib

cp /etc/ts.conf -d /etc

cp -r lib/ts /lib

cp bin/* /bin
```

#### 10 关闭当前GUI

```shell
mv /etc/init.d/S05lvgl /root

mv /etc/init.d/S99myirhmi2 /root

reboot
```

#### 11 运行测试程序

```
ts_print
或
ts_test_mt
```

