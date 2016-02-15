ap.CFLAGS += -DUSE_CANARD -DUSE_CAN_EXT_ID

$(TARGET).srcs   += mcu_periph/can.c
$(TARGET).srcs   += $(SRC_ARCH)/mcu_periph/can_arch.c
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/libcanard/canard.c
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/node.c

$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/libcanard/actuator/canard_actuators.c

#$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/canard_internals.h