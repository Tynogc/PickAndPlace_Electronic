PROJECT = smartFeeder
BUILD_DIR = bin

CFILES = main.c fixed_decimal_math.c handler.c motor.c printf.c usart.c ws2812.c
LDLIBS = -lm

# TODO - you will need to edit these two lines!
DEVICE=stm32f042K6T6
#OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
