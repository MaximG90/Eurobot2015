MAKE = make -s
TARGET = mainboard

# repertoire des modules
AVERSIVE_DIR = ../aversive

SRC  = actuator.c
SRC += ax12_user.c
SRC += bootloader.c
SRC += cmdline.c
SRC += commands/commands.c
SRC += commands/commands_ax12.c
SRC += commands/commands_cs.c
SRC += commands/commands_gen.c
SRC += commands/commands_mainboard.c
SRC += commands/commands_strat.c
SRC += commands/commands_traj.c
SRC += cs.c
SRC += eeprom.c
SRC += lift.c
SRC += main.c
SRC += motion.c
SRC += obstacle.c
SRC += sensor.c
SRC += strat.c


ASRC =

CFLAGS += -I./config
CFLAGS += -Wall -DHOST_AVR
CFLAGS += -g -mmcu=$(MCU) -Wall -Wstrict-prototypes -Os -mcall-prologues -Wno-deprecated-declarations -D__PROG_TYPES_COMPAT__
#CFLAGS += -Werror
#CFLAGS += -DHOMOLOGATION

# link option
LDFLAGS = -T ../common/avr6.x

########################################

-include .aversive_conf
include $(AVERSIVE_DIR)/mk/aversive_project.mk

AVRDUDE_DELAY=50

prog_flash: $(TARGET).$(FORMAT_EXTENSION) $(TARGET).eep
	$(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)

prog_eeprom:

prog_fuse:
