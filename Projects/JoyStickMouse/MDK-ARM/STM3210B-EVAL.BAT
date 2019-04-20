SET PATH=D:\Keil5\ARM\ARMCC\Bin;C:\ProgramData\Oracle\Java\javapath;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\Program Files\Microsoft SQL Server\130\Tools\Binn\;C:\Program Files\dotnet\;C:\Program Files (x86)\Windows Kits\10\Windows Performance Toolkit\;%ANDROID_HOME%;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\Microsoft VS Code\bin;D:\Git\cmd;C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility;D:\Anaconda3;D:\Anaconda3\Library\mingw-w64\bin;D:\Anaconda3\Library\usr\bin;D:\Anaconda3\Library\bin;D:\Anaconda3\Scripts;C:\Program Files (x86)\Atmel\sam-ba_2.12\drv\;C:\Program Files (x86)\Atmel\sam-ba_2.12;C:\Users\Haven\AppData\Local\Microsoft\WindowsApps;D:\dev\nvm;D:\dev\nodejs;D:\Git\cmd;D:\dev\nvm\npm;D:\jdk\bin;D:\android\tools;D:\android\platform-tools;C:\Users\Haven\AppData\Local\Programs\Fiddler;C:\Users\Haven\AppData\Local\Microsoft\WindowsApps;C:\Program Files\Microsoft VS Code\bin;D:\cmder;
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F103VB
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM3210B-EVAL
SET CPU_CLOCK=0x007A1200
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\misc.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\hw_config.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\main.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_desc.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_istr.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_prop.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_pwr.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_endp.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32_it.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\system_stm32f10x.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmAsm" --Via ".\stm3210b-eval\startup_stm32f10x_md._ia"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_core.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_init.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_int.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_mem.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_regs.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\usb_sil.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_pwr.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_rcc.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_exti.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_gpio.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_i2c.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_usart.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_flash.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_spi.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_dma.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm32f10x_sdio.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmCC" --Via ".\stm3210b-eval\stm3210b_eval.__i"
"D:\Keil5\ARM\ARMCC\Bin\ArmLink" --Via ".\STM3210B-EVAL\STM3210B-EVAL.lnp"
"D:\Keil5\ARM\ARMCC\Bin\fromelf.exe" ".\STM3210B-EVAL\STM3210B-EVAL.axf" --i32combined --output ".\STM3210B-EVAL\STM3210B-EVAL.hex"
