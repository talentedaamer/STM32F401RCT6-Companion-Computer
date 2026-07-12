# List of all the board related files.
# BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_F401RE/board.c
# Required include directories
# BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_F401RE

# BOARDSRC = $(PROJECT_DIR)/board/board.c
# BOARDINC = $(PROJECT_DIR)/board

BOARDSRC = ./board/board.c
BOARDINC = ./board

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
