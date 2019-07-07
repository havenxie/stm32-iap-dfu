SET PATH=D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin;D:\Program Files\Keil_v5\ARM\ARMCC\Bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\Intel\WiFi\bin\;C:\Program Files\Common Files\Intel\WirelessCommon\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;C:\Program Files\dotnet\;C:\Program Files\Microsoft SQL Server\130\Tools\Binn\;D:\Program Files (x86)\IncrediBuild;C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility;D:\Program Files\Git\cmd;C:\Users\haven\AppData\Local\Microsoft\WindowsApps;D:\Program Files\Microsoft VS Code\bin
SET CPU_TYPE=STM32F103VB
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM3210B-EVAL
SET CPU_CLOCK=0x00B71B00
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\misc.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\hw_config.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\main.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_desc.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_istr.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_prop.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_pwr.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_endp.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32_it.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\system_stm32f10x.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmAsm" --Via ".\stm3210b-eval\startup_stm32f10x_md._ia"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_core.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_init.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_int.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_mem.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_regs.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\usb_sil.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_pwr.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_rcc.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_exti.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_gpio.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_i2c.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_usart.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_flash.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_spi.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_dma.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm32f10x_sdio.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\stm3210b-eval\stm3210b_eval.__i"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\ArmLink" --Via ".\STM3210B-EVAL\STM3210B-EVAL.lnp"
"D:\Program Files\Keil_v5\ARM\ARMCLANG\Bin\fromelf.exe" ".\STM3210B-EVAL\STM3210B-EVAL.axf" --i32combined --output ".\STM3210B-EVAL\STM3210B-EVAL.hex"
./../../../Utilities/Binary/hex2bin.exe ./STM3210B-EVAL/STM3210B-EVAL.hex
./../../../Utilities/Binary/hex2dfu.exe -t ./STM3210B-EVAL/STM3210B-EVAL.hex
