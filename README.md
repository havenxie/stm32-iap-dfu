# stm32-iap-dfu
STM32的USB DFU升级，包括BOOT部分的Device_Firmware_Upgrade和App部分的Custom_HID。

最新进展：

1. 报告描述符调试完成。
2. 支持DfuSeDemo.exe和USB HID Demonstrator(v1.0.2).exe。（这两个上位机exe可以在ST官网上下载，也可以像我索要）
3. BOOT和App协同应用可支持 HID detach（通过点击DfuSeDemo_A.exe的“Enter DFU mode”按钮使设备从App区域跳转到DFU模式）
	---- by HavenXie @20190327
4. 修改DfuSeDemo.exe源码使之具有DfuSeDemo_A.exe功能

next：Virtual_COM_Port支持DFU


