DIR_INC = ./include
DIR_SRC = ./
DIR_OBJ = ./
DIR_BIN = ./

SRC = $(wildcard ${DIR_SRC}/*.c)
OBJ = $(patsubst %.c,${DIR_OBJ}/%.o,$(notdir ${SRC})) 
DEPS = $(patsubst %.c, ${DIR_OBJ}/%.d, $(notdir ${SRC}))

TARGET = v4l2-demo

BIN_TARGET = ${DIR_BIN}/${TARGET}
 
CC = arm-linux-gnueabihf-gcc
CFLAGS = -g -Wall -I${DIR_INC}
LIB = 

${BIN_TARGET}:${OBJ}
	$(CC) $(OBJ) -o $@ $(LIB)
${DIR_OBJ}/%.o:${DIR_SRC}/%.c
	$(CC) $(CFLAGS) -c  $< -o $@ -MMD -MF ${DIR_OBJ}/$*.d
.PHONY:clean
clean:
	find ${DIR_OBJ} -name '*.o' -exec rm -rf {} \;
	find ${DIR_OBJ} -name '*.d'    -exec rm -rf {} \;
-include $(DEPS)