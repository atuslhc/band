1. 这个版本发行版

2. 结构：加载代码驻留在 FLASH 的 0x00k开始的地址上，运行长度为28K （ 0x7000);  启动后的 boot完成简单的代码加载过程
   把 bootloader 的 image 文件从 FLASH（也在 0x00-0x7000  ) 搬移到RAM  0x2000000 开始的地址上，然后跳转到RAM，执行 bootloader

    bootloader  的任务就是 把FLASH空间的数据 从 0x70000 开始的整个空间做CRC检查，然后读取存放在FLASH末尾的四个字节的CRC ，两个CRC

    进行比较，如果相等，就跳转到 0x7000  位置 执行APP代码 ；否则就打开USB，用HID与PC连接 完成代码的升级动作

3.  所以APP的编译地址 是从 0x7000 开始， 需要配置好APP代码的  icf 

4.  注意 bootloader的 main.c的第一行代码  SCB->VTOR = 0x20000000; ，保证中断的入口地址始终在RAM的起始位置

5.  预编译选项中的 “ewarm” 参数很重要，不然会造成内存对齐的问题

6.  bootloader的代码是采用H文件的形式存放在boot工程里，这样就烧录到了FLASH的  0x00-0x7000 ,供 boot 去加载, 编译好的bootloaer .bin , 用ArrayConvert.exe
  转换成 .H 文件，然后粘贴这个数组包括大小 覆盖Bootld 里面的 bootld.h 中的 bootloader[22256] = {}

7. APP对应的代码：
     A：编译地址的改到 0x7000
     B: 接收PC 升级命令后 ，把FLASH最后的四个字节擦除掉，然后reset


ps:将USB-TTL线接到PD0-TX，PD1-RX，通过串口调试助手115200-8-1-N可输出调试信息。


注意：I4的bootloader主要是对bootloadersize进行了改正，改成38K，APP编译的起始地址要设置成0x9800

