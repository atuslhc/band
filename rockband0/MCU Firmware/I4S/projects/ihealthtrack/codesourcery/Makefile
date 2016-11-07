####################################################################
# Makefile                                                         #
####################################################################

.SUFFIXES:				# ignore builtin rules
.PHONY: all debug release clean

####################################################################
# Definitions                                                      #
####################################################################

DEVICE = EFM32GG330F1024
PROJECTNAME = ihealthtrack

OBJ_DIR = build
EXE_DIR = exe
LST_DIR = lst

####################################################################
# Definitions of toolchain.                                        #
# You might need to do changes to match your system setup          #
####################################################################

# Change path to CodeSourcery tools according to your system configuration
# DO NOT add trailing whitespace chars, they do matter !
WINDOWSCS  ?= C:\Program Files\CodeSourcery\Sourcery_CodeBench_Lite_for_ARM_EABI
LINUXCS    ?= /cad/codesourcery/arm-none-eabi/cs-lite


RM         := rm -rf
NULLDEVICE := /dev/null
SHELLNAMES := $(ComSpec)$(COMSPEC)

# Try autodetecting the environment
ifeq ($(SHELLNAMES),)
  # Assume we are making on a Linux platform
  TOOLDIR := $(LINUXCS)
else
  QUOTE :="
  ifneq ($(COMSPEC),)
    # Assume we are making on a mingw/msys/cygwin platform running on Windows
    # This is a convenient place to override TOOLDIR, DO NOT add trailing
    # whitespace chars, they do matter !
    TOOLDIR := $(WINDOWSCS)
    ifeq ($(findstring cygdrive,$(shell set)),)
      # We were not on a cygwin platform
      NULLDEVICE := NUL
    endif
  else
    # Assume we are making on a Windows platform
    # This is a convenient place to override TOOLDIR, DO NOT add trailing
    # whitespace chars, they do matter !
    SHELL      := $(SHELLNAMES)
    TOOLDIR    := $(WINDOWSCS)
    RM         := "$(TOOLDIR)/bin/cs-rm" -rf
    NULLDEVICE := NUL
  endif
endif

# Create directories and do a clean which is compatible with parallell make
$(shell mkdir $(OBJ_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(EXE_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(LST_DIR)>$(NULLDEVICE) 2>&1)
ifeq (clean,$(findstring clean, $(MAKECMDGOALS)))
  ifneq ($(filter $(MAKECMDGOALS),all debug release),)
    $(shell $(RM) $(OBJ_DIR)/*>$(NULLDEVICE) 2>&1)
    $(shell $(RM) $(EXE_DIR)/*>$(NULLDEVICE) 2>&1)
    $(shell $(RM) $(LST_DIR)/*>$(NULLDEVICE) 2>&1)
  endif
endif

CC      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-gcc$(QUOTE)
LD      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-ld$(QUOTE)
AR      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-ar$(QUOTE)
OBJCOPY = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-objcopy$(QUOTE)
DUMP    = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-objdump$(QUOTE)

####################################################################
# Flags                                                            #
####################################################################

# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = -MMD -MP -MF $(@:.o=.d)

#
# Add -Wa,-ahld=$(LST_DIR)/$(@F:.o=.lst) to CFLAGS to produce assembly list files
#
override CFLAGS += -D$(DEVICE) -Wall -Wextra -Wno-missing-field-initializers \
-mcpu=cortex-m3 -mthumb -ffunction-sections  -fshort-wchar \
-fdata-sections -mfix-cortex-m3-ldrd -fomit-frame-pointer -DDEBUG_EFM       \
$(DEPFLAGS)

override ASMFLAGS += -x assembler-with-cpp -D$(DEVICE) -DCNV_BOARD -Wall -Wextra -mcpu=cortex-m3 -mthumb -fshort-wchar

#
# NOTE: The -Wl,--gc-sections flag may interfere with debugging using gdb.
#
override LDFLAGS += -Xlinker -Map=$(LST_DIR)/$(PROJECTNAME).map -mcpu=cortex-m3 \
-mthumb -T../../../mcu/arm/EFM32/Device/EnergyMicro/EFM32GG/Source/G++/efm32gg.ld \
 -Wl,--gc-sections -Wl,--no-wchar-size-warning

LIBS = -Wl,--start-group -lgcc -lc -lcs3 -lcs3unhosted  \
../../../gui/emwin/lib/libemwin_GCC_CM3.a ../../common/drivers/touch-lib.a -Wl,--end-group

INCLUDEPATHS += \
-I../src \
-I../src/memlcdDriver \
-I../../common/drivers \
-I../../../gui/emwin/inc \
-I../../../gui/emwin/portable \
-I../../../usb/emusb/inc \
-I../../../os/FreeRTOS/include \
-I../../../os/FreeRTOS/portable/GCC/ARM_CM3 \
-I../../../os/FreeRTOS/portable/CMSIS \
-I../../../mcu/arm/CMSIS/Include \
-I../../../mcu/arm/EFM32/Device/EnergyMicro/EFM32GG/Include \
-I../../../mcu/arm/EFM32/emlib/inc 

####################################################################
# Files                                                            #
####################################################################
C_OS_SRC += \
../../../os/FreeRTOS/croutine.c \
../../../os/FreeRTOS/list.c \
../../../os/FreeRTOS/queue.c \
../../../os/FreeRTOS/tasks.c \
../../../os/FreeRTOS/timers.c \
../../../os/FreeRTOS/portable/MemMang/heap_2.c \
../../../os/FreeRTOS/portable/GCC/ARM_CM3/port.c \
../../../os/FreeRTOS/portable/CMSIS/cmsis_os.c \
../../../os/FreeRTOS/portable/LowPower/lowpower_efm32.c 

C_MCU_SRC += \
../../../mcu/arm/EFM32/Device/EnergyMicro/EFM32GG/Source/system_efm32gg.c \
../../../mcu/arm/EFM32/emlib/src/em_assert.c \
../../../mcu/arm/EFM32/emlib/src/em_system.c \
../../../mcu/arm/EFM32/emlib/src/em_rtc.c \
../../../mcu/arm/EFM32/emlib/src/em_burtc.c \
../../../mcu/arm/EFM32/emlib/src/em_cmu.c \
../../../mcu/arm/EFM32/emlib/src/em_rmu.c \
../../../mcu/arm/EFM32/emlib/src/em_int.c \
../../../mcu/arm/EFM32/emlib/src/em_gpio.c \
../../../mcu/arm/EFM32/emlib/src/em_timer.c \
../../../mcu/arm/EFM32/emlib/src/em_letimer.c \
../../../mcu/arm/EFM32/emlib/src/em_dma.c \
../../../mcu/arm/EFM32/emlib/src/em_usart.c \
../../../mcu/arm/EFM32/emlib/src/em_emu.c \
../../../mcu/arm/EFM32/emlib/src/em_acmp.c \
../../../mcu/arm/EFM32/emlib/src/em_lesense.c 

C_BSP_SRC += \
../../common/drivers/beep.c \
../../common/drivers/clockApp.c \
../../common/drivers/clock.c \
../../common/drivers/leds.c \
../../common/drivers/bsp.c \
../../common/drivers/button.c \
../../common/drivers/caplesense.c \
../../common/drivers/dmactrl.c 

C_APP_SRC += \
../src/GlobalData.c \
../src/interrupt.c \
../src/clock_task.c \
../src/touch_task.c \
../src/display_task.c \
../src/usb_app.c \
../src/main.c

C_USB_SRC += \
../../../usb/EMUSB/src/em_usbd.c \
../../../usb/EMUSB/src/em_usbdch9.c \
../../../usb/EMUSB/src/em_usbdep.c \
../../../usb/EMUSB/src/em_usbdint.c \
../../../usb/EMUSB/src/em_usbhal.c

C_GUI_SRC += \
../../../gui/emwin/portable/GUI_X.c \
../../../gui/emwin/portable/GUIConf.c \
../../../gui/emwin/portable/LCDConf.c \
../src/GUI/FramewinDLG.c \
../src/GUI/pedometerDLG.c \
../src/GUI/TemperatureDLG.c \
../src/GUI/WindowDLG.c \
../src/memlcdDriver/dma.c \
../src/memlcdDriver/framebufferctrl.c \
../src/memlcdDriver/memlcd.c

C_SRC = $(C_OS_SRC) $(C_MCU_SRC) $(C_APP_SRC) $(C_USB_SRC) $(C_GUI_SRC) $(C_BSP_SRC)

s_SRC +=  \
../../../mcu/arm/EFM32/Device/EnergyMicro/EFM32GG/Source/G++/startup_efm32gg.s

S_SRC += 

####################################################################
# Rules                                                            #
####################################################################

C_FILES = $(notdir $(C_SRC) )
S_FILES = $(notdir $(S_SRC) $(s_SRC) )
#make list of source paths, sort also removes duplicates
C_PATHS = $(sort $(dir $(C_SRC) ) )
S_PATHS = $(sort $(dir $(S_SRC) $(s_SRC) ) )

C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.o))
S_OBJS = $(if $(S_SRC), $(addprefix $(OBJ_DIR)/, $(S_FILES:.S=.o)))
s_OBJS = $(if $(s_SRC), $(addprefix $(OBJ_DIR)/, $(S_FILES:.s=.o)))
C_DEPS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.d))
OBJS = $(C_OBJS) $(S_OBJS) $(s_OBJS)

vpath %.c $(C_PATHS)
vpath %.s $(S_PATHS)
vpath %.S $(S_PATHS)

# Default build is debug build
all:      test debug
test:

debug:    CFLAGS += -DDEBUG -O0 -g3 -gdwarf-2
debug:    ASMFLAGS += -DDEBUG -O0 -g3 -gdwarf-2
debug:    $(EXE_DIR)/$(PROJECTNAME).bin

release:  CFLAGS += -DNDEBUG -Os -g3 
release:  $(EXE_DIR)/$(PROJECTNAME).bin

# Create objects from C SRC files
$(OBJ_DIR)/%.o: %.c
	@echo "Building file: $<"
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Assemble .s/.S files
$(OBJ_DIR)/%.o: %.s
	@echo "Assembling $<"
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

$(OBJ_DIR)/%.o: %.S
	@echo "Assembling $<"
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Link
$(EXE_DIR)/$(PROJECTNAME).out: $(OBJS)
	@echo "Linking target: $@"
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $(EXE_DIR)/$(PROJECTNAME).out

# Create binary file
$(EXE_DIR)/$(PROJECTNAME).bin: $(EXE_DIR)/$(PROJECTNAME).out
	@echo "Creating binary file"
	$(OBJCOPY) -O binary $(EXE_DIR)/$(PROJECTNAME).out $(EXE_DIR)/$(PROJECTNAME).bin
# Uncomment next line to produce assembly listing of entire program
#	$(DUMP) -h -S -C $(EXE_DIR)/$(PROJECTNAME).out>$(LST_DIR)/$(PROJECTNAME)out.lst

clean:
ifeq ($(filter $(MAKECMDGOALS),all debug release),)
	$(RM) $(OBJ_DIR) $(LST_DIR) $(EXE_DIR)
endif

# include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS)
endif
