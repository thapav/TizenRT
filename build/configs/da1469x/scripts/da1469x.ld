/* Linker script to configure memory regions.
 * Need modifying for a specific board.
 *   RAM.ORIGIN: starting address of read-write RAM area
 *   RAM.LENGTH: length of read-write RAM area
 *   RetRAM0.ORIGIN: starting address of retained RAM area
 *   RetRAM0.LENGTH: length of retained RAM area
 *   BUFFER.ORIGIN: starting address of buffer area
 *   BUFFER.LENGTH: length of buffer area
 */

        MEMORY
        {
                ROM (rx) : ORIGIN = (0x0), LENGTH = (( 1024 * 1024 * 2 ))
                RAM (rw) : ORIGIN = (0x20000000 + 0x200), LENGTH = ((512 * 1024) - 0x200)
                EXT_RAM (rw) : ORIGIN = (0x32060400), LENGTH = (0x20000)
        }
        __FLASH_segment_start__ = ORIGIN( ROM );
        __FLASH_segment_end__ = __FLASH_segment_start__ + LENGTH( ROM );
        __SRAM_segment_start__ = ORIGIN( RAM );
        __SRAM_segment_end__ = __SRAM_segment_start__ + LENGTH( RAM );

/* GROUP(libgcc.a libc.a libm.a libnosys.a) */
ENTRY(Reset_Handler)
SECTIONS
{
        .init_text :
        {
                KEEP(*(.isr_vector))
                . = 0x200;
                __Vectors_End = .;
                __Vectors_Size = __Vectors_End - __isr_vector;
                *(text_reset*)
        } > ROM
        .text :
        {
                *(EXCLUDE_FILE(*libnosys.a:sbrk.o
                               *libgcc.a:_aeabi_uldivmod.o
                               *libgcc.a:_muldi3.o
                               *libgcc.a:_dvmd_tls.o
                               *libgcc.a:bpabi.o
                               *libgcc.a:_udivdi3.o
                               *libgcc.a:_clzdi2.o
                               *libgcc.a:_clzsi2.o
                               *libgcc.a:*.o
                                *libg_nano.a:lib_a-abs*.o
                                *libg_nano.a:lib_a-div*.o
                                *libg_nano.a:lib_a-*print*.o
                                *libg_nano.a:lib_a-mem*.o
                                *libg_nano.a:lib_a-str*.o
                                *libm.a:*.o
                                *qspi_automode.o
                                *hw_dma.o
                                *hw_gpio.o
                                *hw_i2c.o
                                *hw_qspi.o
                                *hw_timer.o
                                *hw_uart.o
                                *msg_queues.o
                                *resmgmt.o
                                *ad_flash.o
                                *ad_i2c.o
                                *ad_uart.o
                                *sys_power_mgr_da1469x.o
                                *sys_timer.o
                                *libarm_ARMv8MMLldfsp_math.a:*.o
                                *sensor_hal.o
                                *algmanager.o
                                *sensorcontroller.o
                                *sensormanager.o
                                *sensor_hrm.o
                                *sensor_hrm_calibration.o
                                *sensor_hrm_control.o
                                *sensor_hrm_eol.o
                                *sensor_accel_lsm6dsl.o
                                *sensor_main.o
                                *AutoSession_Aerobics.o
                                *AutoSession_Elliptical.o
                                *AutoSession_RowingMachine.o
                                *AutoSessionExercise.o
                                *AutoSessionExerciseMonitor_libraryInterface.o
                                *Pedometer_libraryInterface.o
                                *PRI_Pedometer_HS.o
                                *WristUp_libraryInterface.o
                                *WristUp.o
                                *WristDetect.o
                                *WristDetect_libraryInterface.o
                                *APCF.o
                                *DailyHeartRate_libraryInterface.o
                                *HeartRate.o
                                *HeartRate_libraryInterface.o
                                *HRMLibrary.o
                                *HRMLibrary_libraryInterface.o
                                *DynamicWorkOutMonitor.o
                                *InactiveTimer.o
                                *SLMInterface.o
                                *WearOnOffMonitor.o
                                *WearOnOffMonitor_libraryInterface.o
                                *SleepAlgorithm.o
                                *SleepDetectLibrary.o
                                *SleepDetectLibrary_libraryInterface.o
                                *SleepEfficiency.o
                                *SleepMonitor.o
                                *SleepMonitorLibrary.o
                                *SleepMonitorLibrary_libraryInterface.o
                                *SleepStageAnalysis.o
                                *Stress_AlgoMain.o
                                *Stress_AlgoState.o
                                *Stress_AlgoStress.o
                                *Stress_Containner.o
                                *Stress_libraryInterface.o
                                *STRESSLibrary.o
                                ) .text*)
                . = ALIGN(32);
                __cmi_fw_area_start = .;
                KEEP(*(.cmi_fw_area*))
                __cmi_fw_area_end = .;
                __start_adapter_init_section = .;
                KEEP(*(adapter_init_section))
                __stop_adapter_init_section = .;
                __start_bus_init_section = .;
                KEEP(*(bus_init_section))
                __stop_bus_init_section = .;
                __start_device_init_section = .;
                KEEP(*(device_init_section))
                __stop_device_init_section = .;
                KEEP(*(.init))
                KEEP(*(.fini))
                *crtbegin.o(.ctors)
                *crtbegin?.o(.ctors)
                *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
                *(SORT(.ctors.*))
                *(.ctors)
                *crtbegin.o(.dtors)
                *crtbegin?.o(.dtors)
                *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
                *(SORT(.dtors.*))
                *(.dtors)
                . = ALIGN(4);
                PROVIDE_HIDDEN (__preinit_array_start = .);
                KEEP(*(.preinit_array))
                PROVIDE_HIDDEN (__preinit_array_end = .);
                . = ALIGN(4);
                PROVIDE_HIDDEN (__init_array_start = .);
                KEEP(*(SORT(.init_array.*)))
                KEEP(*(.init_array))
                PROVIDE_HIDDEN (__init_array_end = .);
                . = ALIGN(4);
                PROVIDE_HIDDEN (__fini_array_start = .);
                KEEP(*(SORT(.fini_array.*)))
                KEEP(*(.fini_array))
                PROVIDE_HIDDEN (__fini_array_end = .);
                *(EXCLUDE_FILE(
                *libg_nano.a:lib_a-abs*.o
                *libg_nano.a:lib_a-div*.o
                *libg_nano.a:lib_a-*print*.o
                *libg_nano.a:lib_a-mem*.o
                *libg_nano.a:lib_a-str*.o
                *libm.a:*.o
                *qspi_automode.o
                *hw_dma.o
                *hw_gpio.o
                *hw_i2c.o
                *hw_qspi.o
                *hw_timer.o
                *hw_uart.o
                *msg_queues.o
                *resmgmt.o
                *ad_flash.o
                *ad_i2c.o
                *ad_uart.o
                *sys_power_mgr_da1469x.o
                *sys_timer.o
                *libarm_ARMv8MMLldfsp_math.a:*.o
                *sensor_hal.o
                *algmanager.o
                *sensorcontroller.o
                *sensormanager.o
                *sensor_hrm.o
                *sensor_hrm_calibration.o
                *sensor_hrm_control.o
                *sensor_hrm_eol.o
                *sensor_accel_lsm6dsl.o
                *sensor_main.o
                *AutoSession_Aerobics.o
                *AutoSession_Elliptical.o
                *AutoSession_RowingMachine.o
                *AutoSessionExercise.o
                *AutoSessionExerciseMonitor_libraryInterface.o
                *Pedometer_libraryInterface.o
                *PRI_Pedometer_HS.o
                *WristUp_libraryInterface.o
                *WristUp.o
                *WristDetect.o
                *WristDetect_libraryInterface.o
                *APCF.o
                *DailyHeartRate_libraryInterface.o
                *HeartRate.o
                *HeartRate_libraryInterface.o
                *HRMLibrary.o
                *HRMLibrary_libraryInterface.o
                *DynamicWorkOutMonitor.o
                *InactiveTimer.o
                *SLMInterface.o
                *WearOnOffMonitor.o
                *WearOnOffMonitor_libraryInterface.o
                *SleepAlgorithm.o
                *SleepDetectLibrary.o
                *SleepDetectLibrary_libraryInterface.o
                *SleepEfficiency.o
                *SleepMonitor.o
                *SleepMonitorLibrary.o
                *SleepMonitorLibrary_libraryInterface.o
                *SleepStageAnalysis.o
                *Stress_AlgoMain.o
                *Stress_AlgoState.o
                *Stress_AlgoStress.o
                *Stress_Containner.o
                *Stress_libraryInterface.o
                *STRESSLibrary.o
                ) .rodata*)
                KEEP(*(.eh_frame*))
        } > ROM
        .ARM.extab :
        {
                *(.ARM.extab* .gnu.linkonce.armextab.*)
        } > ROM
        __exidx_start = .;
        .ARM.exidx :
        {
                *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        } > ROM
        __exidx_end = .;
        .copy.table :
        {
                . = ALIGN(4);
                __copy_table_start__ = .;
                LONG (__etext)
                LONG (__retention_ram_init_start__)
                LONG ((__retention_ram_init_end__ - __retention_ram_init_start__))
                LONG (__etext + ((__retention_ram_init_end__ - __retention_ram_init_start__)))
                LONG (__snc_section_start__)
                LONG ((__snc_section_end__ - __snc_section_start__))
                LONG (__etext + ((__retention_ram_init_end__ - __retention_ram_init_start__)) + ((__snc_section_end__ - __snc_section_start__)))
                LONG (__non_retention_ram_init_start__)
                LONG ((__non_retention_ram_init_end__ - __non_retention_ram_init_start__))
                LONG (__etext + ((__retention_ram_init_end__ - __retention_ram_init_start__)) + ((__snc_section_end__ - __snc_section_start__)) + ((__non_retention_ram_init_end__ - __non_retention_ram_init_start__)))
                LONG (__data_ext_ram_start__)
                LONG ((__data_ext_ram_end__ - __data_ext_ram_start__))
                __copy_table_end__ = .;
        } > ROM
        .zero.table :
        {
                . = ALIGN(4);
                __zero_table_start__ = .;
                LONG (__bss_start__)
                LONG (__bss_end__ - __bss_start__)
                LONG (__retention_ram_zi_start__)
                LONG (__retention_ram_zi_end__ - __retention_ram_zi_start__)
                LONG (__bss_ext_ram_start__)
                LONG (__bss_ext_ram_end__ - __bss_ext_ram_start__)
                __zero_table_end__ = .;
        } > ROM
        __etext = .;
        .retention_ram_uninit (NOLOAD) :
        {
                __retention_ram_uninit_start__ = .;
                ASSERT( . == ORIGIN(RAM), ".retention_ram_uninit section moved!");
                KEEP(*(nmi_info))
                KEEP(*(hard_fault_info))
                KEEP(*(retention_mem_uninit))
                ASSERT( . <= __retention_ram_uninit_start__ + (128+3020),
                        "retention_ram_uninit section overflowed! Increase dg_config_RETAINED_UNINIT_SECTION_SIZE.");
                . = __retention_ram_uninit_start__ + (128+3020);
                __retention_ram_uninit_end__ = .;
        } > RAM
        .retention_ram_init : AT (0x0 +__etext)
        {
                __retention_ram_init_start__ = .;
                . = ALIGN(32);
                __unprivileged_readonly_start__ = .;
                __unprivileged_text_start__ = .;
                *(text_retained)
                *libnosys.a:sbrk.o (.text*)
                *libgcc.a:_aeabi_uldivmod.o (.text*)
                *libgcc.a:_muldi3.o (.text*)
                *libgcc.a:_dvmd_tls.o (.text*)
                *libgcc.a:bpabi.o (.text*)
                *libgcc.a:_udivdi3.o (.text*)
                *libgcc.a:_clzdi2.o (.text*)
                *libgcc.a:_clzsi2.o (.text*)
                *libgcc.a:*.o (.text*)
                *libg_nano.a:lib_a-abs*.o (.text*)
                *libg_nano.a:lib_a-div*.o (.text*)
                *libg_nano.a:lib_a-*print*.o (.text*)
                *libg_nano.a:lib_a-mem*.o (.text*)
                *libg_nano.a:lib_a-str*.o (.text*)
                *libm.a:*.o (.text*)
                *qspi_automode.o (.text*)
                *hw_dma.o (.text*)
                *hw_gpio.o (.text*)
                *hw_i2c.o (.text*)
                *hw_qspi.o (.text*)
                *hw_timer.o (.text*)
                *hw_uart.o (.text*)
                *msg_queues.o (.text*)
                *resmgmt.o (.text*)
                *ad_flash.o (.text*)
                *ad_i2c.o (.text*)
                *ad_uart.o (.text*)
                *sys_power_mgr_da1469x.o (.text*)
                *sys_timer.o (.text*)
                *libarm_ARMv8MMLldfsp_math.a:*.o (.text*)
                *sensor_hal.o (.text*)
                *algmanager.o (.text*)
                *sensorcontroller.o (.text*)
                *sensormanager.o (.text*)
                *sensor_hrm.o (.text*)
                *sensor_hrm_calibration.o (.text*)
                *sensor_hrm_control.o (.text*)
                *sensor_hrm_eol.o (.text*)
                *sensor_accel_lsm6dsl.o (.text*)
                *sensor_main.o (.text*)
                *AutoSession_Aerobics.o (.text*)
                *AutoSession_Elliptical.o (.text*)
                *AutoSession_RowingMachine.o (.text*)
                *AutoSessionExercise.o (.text*)
                *AutoSessionExerciseMonitor_libraryInterface.o (.text*)
                *Pedometer_libraryInterface.o (.text*)
                *PRI_Pedometer_HS.o (.text*)
                *WristUp_libraryInterface.o (.text*)
                *WristUp.o (.text*)
                *WristDetect.o (.text*)
                *WristDetect_libraryInterface.o (.text*)
                *APCF.o (.text*)
                *DailyHeartRate_libraryInterface.o (.text*)
                *HeartRate.o (.text*)
                *HeartRate_libraryInterface.o (.text*)
                *HRMLibrary.o (.text*)
                *HRMLibrary_libraryInterface.o (.text*)
                *DynamicWorkOutMonitor.o (.text*)
                *InactiveTimer.o (.text*)
                *SLMInterface.o (.text*)
                *WearOnOffMonitor.o (.text*)
                *WearOnOffMonitor_libraryInterface.o (.text*)
                *SleepAlgorithm.o (.text*)
                *SleepDetectLibrary.o (.text*)
                *SleepDetectLibrary_libraryInterface.o (.text*)
                *SleepEfficiency.o (.text*)
                *SleepMonitor.o (.text*)
                *SleepMonitorLibrary.o (.text*)
                *SleepMonitorLibrary_libraryInterface.o (.text*)
                *SleepStageAnalysis.o (.text*)
                *Stress_AlgoMain.o (.text*)
                *Stress_AlgoState.o (.text*)
                *Stress_AlgoStress.o (.text*)
                *Stress_Containner.o (.text*)
                *Stress_libraryInterface.o (.text*)
                *STRESSLibrary.o (.text*)
                . = ALIGN(32);
                __unprivileged_text_end__ = .;
                *libg_nano.a:lib_a-abs*.o (.rodata*)
                *libg_nano.a:lib_a-div*.o (.rodata*)
                *libg_nano.a:lib_a-*print*.o (.rodata*)
                *libg_nano.a:lib_a-mem*.o (.rodata*)
                *libg_nano.a:lib_a-str*.o (.rodata*)
                *libm.a:*.o (.rodata*)
                *qspi_automode.o (.rodata*)
                *hw_dma.o (.rodata*)
                *hw_gpio.o (.rodata*)
                *hw_i2c.o (.rodata*)
                *hw_qspi.o (.rodata*)
                *hw_timer.o (.rodata*)
                *hw_uart.o (.rodata*)
                *msg_queues.o (.rodata*)
                *resmgmt.o (.rodata*)
                *ad_flash.o (.rodata*)
                *ad_i2c.o (.rodata*)
                *ad_uart.o (.rodata*)
                *sys_power_mgr_da1469x.o (.rodata*)
                *sys_timer.o (.rodata*)
                *libarm_ARMv8MMLldfsp_math.a:*.o (.rodata*)
                *sensor_hal.o (.rodata*)
                *algmanager.o (.rodata*)
                *sensorcontroller.o (.rodata*)
                *sensormanager.o (.rodata*)
                *sensor_hrm.o (.rodata*)
                *sensor_hrm_calibration.o (.rodata*)
                *sensor_hrm_control.o (.rodata*)
                *sensor_hrm_eol.o (.rodata*)
                *sensor_accel_lsm6dsl.o (.rodata*)
                *sensor_main.o (.rodata*)
                *AutoSession_Aerobics.o (.rodata*)
                *AutoSession_Elliptical.o (.rodata*)
                *AutoSession_RowingMachine.o (.rodata*)
                *AutoSessionExercise.o (.rodata*)
                *AutoSessionExerciseMonitor_libraryInterface.o (.rodata*)
                *Pedometer_libraryInterface.o (.rodata*)
                *PRI_Pedometer_HS.o (.rodata*)
                *WristUp_libraryInterface.o (.rodata*)
                *WristUp.o (.rodata*)
                *WristDetect.o (.rodata*)
                *WristDetect_libraryInterface.o (.rodata*)
                *APCF.o (.rodata*)
                *DailyHeartRate_libraryInterface.o (.rodata*)
                *HeartRate.o (.rodata*)
                *HeartRate_libraryInterface.o (.rodata*)
                *HRMLibrary.o (.rodata*)
                *HRMLibrary_libraryInterface.o (.rodata*)
                *DynamicWorkOutMonitor.o (.rodata*)
                *InactiveTimer.o (.rodata*)
                *SLMInterface.o (.rodata*)
                *WearOnOffMonitor.o (.rodata*)
                *WearOnOffMonitor_libraryInterface.o (.rodata*)
                *SleepAlgorithm.o (.rodata*)
                *SleepDetectLibrary.o (.rodata*)
                *SleepDetectLibrary_libraryInterface.o (.rodata*)
                *SleepEfficiency.o (.rodata*)
                *SleepMonitor.o (.rodata*)
                *SleepMonitorLibrary.o (.rodata*)
                *SleepMonitorLibrary_libraryInterface.o (.rodata*)
                *SleepStageAnalysis.o (.rodata*)
                *Stress_AlgoMain.o (.rodata*)
                *Stress_AlgoState.o (.rodata*)
                *Stress_AlgoStress.o (.rodata*)
                *Stress_Containner.o (.rodata*)
                *Stress_libraryInterface.o (.rodata*)
                *STRESSLibrary.o (.rodata*)
                . = ALIGN(32);
                __unprivileged_readonly_end__ = .;
                *(privileged_data_init)
                *(.retention)
                *(vtable)
                *(retention_mem_init)
                *(retention_mem_const)
                *libg_nano.a:* (.data*)
                *libnosys.a:* (.data*)
                *libgcc.a:* (.data*)
                *libble_stack_d2522_00.a:* (.data*)
                *libble_stack_da1469x.a:* (.data*)
                *libad9361_radio.a:* (.data*)
                *crtbegin.o (.data*)
                KEEP(*(.jcr*))
                . = ALIGN(4);
                __retention_ram_init_end__ = .;
        } > RAM
        .retention_ram_zi (COPY) :
        {
                __retention_ram_zi_start__ = .;
                *(privileged_data_zi)
                *(retention_mem_zi)
                *libg_nano.a:* (.bss*)
                *libnosys.a:* (.bss*)
                *libgcc.a:* (.bss*)
                *libble_stack_d2522_00.a:* (.bss*)
                *libble_stack_da1469x.a:* (.bss*)
                *libad9361_radio.a:* (.bss*)
                *crtbegin.o (.bss*)
                __OsHeapBase__ = .;
                *(os_heap)
                __OsHeapLimit__ = .;
                __HeapBase = .;
                __end__ = .;
                end = __end__;
                KEEP(*(.heap*))
                __HeapLimit = .;
                __retention_ram_zi_end__ = .;
                . = ALIGN(32);
                __mpu_protection_1_start__ = .;
                *(mpu_pretect_1)
                . = ALIGN(32);
                __mpu_protection_1_end__ = .;
        } > RAM
        .stack_section (NOLOAD) :
        {
                . = . + 1;
                . = ALIGN(8);
                __StackLimit = .;
                KEEP(*(.stack*))
                . = ALIGN(8);
                __StackTop = .;
                PROVIDE(__stack = __StackTop);
                . = ALIGN(32);
                __mpu_protection_2_start__ = .;
                *(mpu_pretect_2)
                . = ALIGN(32);
                __mpu_protection_2_end__ = .;
        } > RAM
        .snc_section : AT (0x0 + __etext + ((__retention_ram_init_end__ - __retention_ram_init_start__)))
        {
                __snc_section_start__ = .;
                . = ALIGN(4);
                KEEP(*(.snc_region*))
                . = ALIGN(4);
                __snc_section_end__ = .;
        } > RAM
        .cmi_section (NOLOAD) :
        {
                __cmi_section_start__ = .;
                . = ALIGN(0x400);
                cmi_fw_dst_addr = .;
                . += (__cmi_fw_area_end - __cmi_fw_area_start);
                . = ALIGN(0x400);
                __cmi_section_end__ = . - 1;
        } > RAM
        __non_retention_ram_start__ = .;
        .non_retention_ram_init : AT (0x0 + __etext + ((__retention_ram_init_end__ - __retention_ram_init_start__)) + ((__snc_section_end__ - __snc_section_start__)))
        {
                __non_retention_ram_init_start__ = .;
                . = ALIGN(4);
                *(EXCLUDE_FILE(*libg_nano.a:* *libnosys.a:* *libgcc.a:* *libble_stack_d2522_00.a:* *libble_stack_da1469x.a:* *libad9361_radio.a:* *crtbegin.o
                    *ui*.o
                    *libemwin.a:*BIDI*.o
                    *libemwin.a:*BiDi*.o
                    *datamanager*.o
                    *nfc*.o
                    *wireless_charging*.o
                    *SensorLoggingUtil.o
                    *swimming*.o
                    *sensor_gyro_lsm6dsl.o
                    *libPCA9431-DA14697.a:*.o
                    *libWlcDeviceLibrary-NT3H2211-DA14697.a:*.o
                    *sec_mpu*.o) .data*)
                . = ALIGN(4);
                __non_retention_ram_init_end__ = .;
        } > RAM
        .bss :
        {
                . = ALIGN(4);
                __bss_start__ = .;
                                *(EXCLUDE_FILE(*libg_nano.a:* *libnosys.a:* *libgcc.a:* *libble_stack_d2522_00.a:* *libble_stack_da1469x.a:* *libad9361_radio.a:* *crtbegin.o
                    *ui*.o
                    *libemwin.a:*BIDI*.o
                    *libemwin.a:*BiDi*.o
                    *sap_lite*.o
                    *datamanager*.o
                    *nfc*.o
                    *wireless_charging*.o
                    *bigdata*.o
                    *AT_cmd_task*.o
                    *libWlcDeviceLibrary-NT3H2211-DA14697.a:*.o
                    *caloriecore*.o
                    *swimming*.o
                    *swimming_libraryinterface*.o
                    *exercise_libraryinterface*.o
                    *SleepStageAnalysis.o
                    *SensorLoggingUtil.o
                    *Exercise.o
                    *Exercise_libraryInterface.o
                    *performance*.o
                    *logdump_task*.o
                    *ble_peripheral_task*.o
                    *sec_batt_adapter*.o
                    *sensor_gyro_lsm6dsl.o) .bss*)
                *(EXCLUDE_FILE(
                    *ui*.o
                    *libemwin.a:*BIDI*.o
                    *libemwin.a:*BiDi*.o
                    *sap_lite*.o
                    *datamanager*.o
                    *nfc*.o
                    *wireless_charging*.o
                    *bigdata*.o
                    *AT_cmd_task*.o
                    *libWlcDeviceLibrary-NT3H2211-DA14697.a:*.o
                    *caloriecore*.o
                    *swimming*.o
                    *swimming_libraryinterface*.o
                    *exercise_libraryinterface*.o
                    *SleepStageAnalysis.o
                    *SensorLoggingUtil.o
                    *Exercise.o
                    *Exercise_libraryInterface.o
                    *performance*.o
                    *logdump_task*.o
                    *ble_peripheral_task*.o
                    *sec_batt_adapter*.o
                    *sensor_gyro_lsm6dsl.o) COMMON)
                . = ALIGN(4);
                __bss_end__ = .;
        } > RAM
        __non_retention_ram_end__ = .;
        __unused_ram_start__ = . + 1;
        __ext_ram_start__ = .;
        .data_ext_ram : AT (0x0 + __etext + ((__retention_ram_init_end__ - __retention_ram_init_start__)) + ((__snc_section_end__ - __snc_section_start__)) + ((__non_retention_ram_init_end__ - __non_retention_ram_init_start__)))
        {
                __data_ext_ram_start__ = .;
                . = ALIGN(4);
                *ui*.o (.data*)
                *libemwin.a:*BIDI*.o (.data*)
                *libemwin.a:*BiDi*.o (.data*)
                *datamanager*.o (.data*)
                *nfc*.o (.data*)
                *wireless_charging*.o (.data*)
                *SensorLoggingUtil.o (.data*)
                *swimming*.o (.data*)
                *sensor_gyro_lsm6dsl.o (.data*)
                *libPCA9431-DA14697.a:*.o (.data*)
                *libWlcDeviceLibrary-NT3H2211-DA14697.a:*.o (.data*)
                *sec_mpu*.o (.data*)
                *(data_ext_ram)
                . = ALIGN(4);
                __data_ext_ram_end__ = .;
        } > EXT_RAM
        .bss_ext_ram :
        {
                __bss_ext_ram_start__ = .;
                . = ALIGN(4);
                *ui*.o (.bss*)
                *libemwin.a:*BIDI*.o (.bss*)
                *libemwin.a:*BiDi*.o (.bss*)
                *sap_lite*.o (.bss*)
                *datamanager*.o (.bss*)
                *nfc*.o (.bss*)
                *wireless_charging*.o (.bss*)
                *bigdata*.o (.bss*)
                *AT_cmd_task*.o (.bss*)
                *libWlcDeviceLibrary-NT3H2211-DA14697.a:*.o (.bss*)
                *caloriecore*.o (.bss*)
                *swimming*.o (.bss*)
                *swimming_libraryinterface*.o (.bss*)
                *exercise_libraryinterface*.o (.bss*)
                *SleepStageAnalysis.o (.bss*)
                *SensorLoggingUtil.o (.bss*)
                *Exercise.o (.bss*)
                *Exercise_libraryInterface.o (.bss*)
                *performance*.o (.bss*)
                *sensor_gyro_lsm6dsl.o (.bss*)
                *logdump_task*.o (.bss*)
                *ble_peripheral_task*.o (.bss*)
                *sec_batt_adapter*.o (.bss*)
                *ui*.o (COMMON)
                *libemwin.a:*BIDI*.o (COMMON)
                *libemwin.a:*BiDi*.o (COMMON)
                *sap_lite*.o (COMMON)
                *datamanager*.o (COMMON)
                *nfc*.o (COMMON)
                *wireless_charging*.o (COMMON)
                *bigdata*.o (COMMON)
                *AT_cmd_task*.o (COMMON)
                *libWlcDeviceLibrary-NT3H2211-DA14697.a:*.o (COMMON)
                *caloriecore*.o (COMMON)
                *swimming*.o (COMMON)
                *swimming_libraryinterface*.o (COMMON)
                *exercise_libraryinterface*.o (COMMON)
                *SleepStageAnalysis.o (COMMON)
                *SensorLoggingUtil.o (COMMON)
                *Exercise.o (COMMON)
                *Exercise_libraryInterface.o (COMMON)
                *performance*.o (COMMON)
                *sensor_gyro_lsm6dsl.o (COMMON)
                *logdump_task*.o (COMMON)
                                *ble_peripheral_task*.o (COMMON)
                *sec_batt_adapter*.o (COMMON)
                *(bss_ext_ram)
                . = ALIGN(4);
                __bss_ext_ram_end__ = .;
        } > EXT_RAM
        __ext_ram_end__ = .;
        ASSERT(__etext + (__retention_ram_init_end__ - __retention_ram_init_start__) + (__snc_section_end__ - __snc_section_start__) + (__non_retention_ram_init_end__ - __non_retention_ram_init_start__) <
                        ORIGIN(ROM) + ( 1024 * 1024 * 2 ), "ROM space overflowed")
}

