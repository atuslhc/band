EFM32 LG/GG USB HID Bootloader
测试硬件及步骤：
1、EFM32 LG STK3500/GG STK3600
2、将USB HID Bootloader压缩包中下位机程序：
\usb-hid-bootloader\boards\EFM32GG990-STK\examples\usbhid\iar路径下的IAR workspace打开（IAR6.50创建）；
3、先编译bootloader工程，产生bin文件，再运行ArrayConvert.exe软件，将生成的bin文件转换为bootld.h文件，并将其添加到Bootld工程中（或替代原有的bootld.h文件）；
4、将编译产生的bin文件下载到STK开发板中。
5、通过USB Device线将STK连接PC端，在芯片上电前，请先按下PB0按键，再上电；
6、PC将识别到USB HID Carder设备；
7、运行上位机软件包中的USBHIDBootloader.exe，点击Read Device将读取芯片的信息；
8、点击load file选择需要升级的App bin文件（App Flash偏移0x7000，即Bootld占用前28K Flash空间），点击Upgrade将进入升级步骤。升级完成，信息提示窗将显示相关信息。

ps:将USB-TTL线接到PD0-TX，PD1-RX，通过串口调试助手115200-8-1-N可输出调试信息。



=============================
1. 在ROM里面调试， 屏蔽 第一行 //SCB->VTOR = 0x20000000;