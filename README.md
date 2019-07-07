# STM32F103Cx-IAP-DFU
STM32F103Cx的USB DFU升级包括BOOT部分App部分：
- BOOT: Device_Firmware_Upgrade
- App ：
    - Custom_HID
	- Custom_HID_VCP

最新进展：
1. 以上3个固件适配X-LINK-A_V2.0硬件
2. 更改App的起始地址和中断向量使之支持DFU
3. 更改App的HID报告描述符使之支持PC端软件USB HID Demonstrator(v1.0.2).exe (该软件可从ST官网下载，也可以向我索要)
4. 修改DfuSeDemo.exe使之支持HID Detach    
5. 更改App的HID报告描述符使之支持HID Detach(通过PC端软件DfuSeDemo发命令使CPU从App区跳转到BOOT区) ----at 20190327
6. 新增复合设备工程Custom_HID_VCP
7. Custom_HID_VCP具有Custom_HID的所有功能并且具有USB虚拟串口的功能 ---- at 20190427
8. 所有工程的xxB项目在Keil5.26版本重新编译并且编译器换成版本6，xxE项目在IAR重新编译。  ----at 20190707

Next Todo：
 - 大容量存储设备支持DFU
 - DfuSeDemo解决“Unknow HID Device”的BUG。
 - DfuSeDemo的command添加更多命令
 - 熟悉DFU流程
 - DFU过程校本化
 - DFU过程采用AES对称加密