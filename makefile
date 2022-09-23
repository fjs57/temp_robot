# CC			?= $(CROSS_COMPILE)g++
CC			= g++
AR			?= $(CROSS_COMPILE)ar
STRIP		?= $(CROSS_COMPILE)strip

SYSTEM = $(shell gcc -dumpmachine | cut -d - -f2)
ARCH = $(shell gcc -dumpmachine | cut -d - -f1)

#-------------------------------------------------------------#
#                      Define directories                     #
#-------------------------------------------------------------#
BUILD_DIR ?= $(abspath ./Build)
OBJ_DIR ?= $(BUILD_DIR)/Obj
BIN_DIR ?= $(BUILD_DIR)/Bin
SUBDIR = $(dir $(wildcard ./Src/*/))

# adds -iquote to every dirs of subdir
INCS = $(addprefix -I , $(SUBDIR))

LDFLAGS = -lpthread #-I /usr/local/include/
CFLAGS = -Wall $(INCS)

OBJS_C=$(patsubst %.cpp, $(OBJ_DIR)/%.o, $(shell find ./Src -name *.cpp))

APPL_NAME=robot 

# all :
# 	@echo $(CFLAGS)

default: all
all : $(APPL_NAME)

robot : $(BIN_DIR)/$(APPL_NAME)
$(BIN_DIR)/$(APPL_NAME): $(OBJS_C)
	@mkdir -p $(@D)
	$(CC) -o $@ $(shell echo $^ | sed -e 's:\./Src/::g') $(LDFLAGS)
	$(STRIP) $@

$(OBJ_DIR)/%.o:	 %.cpp
	@mkdir -p $(shell dirname $(shell echo $@ | sed -e 's,\./Src/,,'))
	$(CC) $(CFLAGS) -o $(shell echo $@ | sed -e 's,\./Src/,,') -c $<
	

clean :
	@rm -Rf $(BUILD_DIR)

mrproper: clean
	@rm -rf $(BIN_DIR)/$(APPL_NAME)

launch:
	Build/Bin/robot

.PHONY: all clean mrproper launch