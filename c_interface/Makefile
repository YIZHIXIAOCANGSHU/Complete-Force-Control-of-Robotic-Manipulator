STM32_DIR = ../stm32_code
STM32_SRCS = $(STM32_DIR)/control_logic.c \
             $(STM32_DIR)/dynamics_lib.c \
             $(STM32_DIR)/kinematics_lib.c \
             $(STM32_DIR)/math_lib.c \
             $(STM32_DIR)/model_lib.c \
             $(STM32_DIR)/trajectory_lib.c \
             $(STM32_DIR)/stm_controller.c

HOST_BRIDGE_SRCS = sim_bridge.c

c_main: main.c main_stm.c sim_interface.c $(STM32_SRCS) $(STM32_DIR)/config.h
	gcc -o c_main main.c main_stm.c sim_interface.c $(HOST_BRIDGE_SRCS) $(STM32_SRCS) -I$(STM32_DIR) -O2 -Wall -lm

serial_gravity_comp: serial_gravity_comp.c main_stm.c $(STM32_SRCS) $(STM32_DIR)/config.h
	gcc -o serial_gravity_comp serial_gravity_comp.c main_stm.c $(STM32_SRCS) -I$(STM32_DIR) -O2 -Wall -lm

clean:
	rm -f c_main serial_gravity_comp
