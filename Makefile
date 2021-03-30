#IncDir			:= ../../zlib ../../gpk/llc ../../stb ../../nanosvg/src
Platform		:= Linux.x64
ProjectName		:= smart
Configuration	:= Release

C_DEFS			:= -DNOMINMAX
OPT				:= 3

ifeq ($(DEBUG), 1)
	Configuration	= Debug
	OPT				= 0
	C_DEFS			+= -DDEBUG
endif

OutDir	= ../$(Platform).$(Configuration)
IntDir	= ../obj/$(Platform).$(Configuration)/$(ProjectName)

OBJDIRS	:= $(patsubst %, $(IntDir), $(ProjectName))
INCDIR	:= $(patsubst %,-I%,$(IncDir))

CFLAGS	:= -std=c++17 -Wall -O$(OPT) $(INCDIR) 
CC		:= gcc

_OBJ	:= $(patsubst %.c,%.o,$(wildcard *.c)) 
OBJ		:= $(patsubst %,$(IntDir)/%,$(_OBJ))

$(IntDir)/%.o: %.c
	$(CC) $(CFLAGS) $(C_DEFS) -c -o $@ $<

build: $(OBJDIRS) $(OBJ)
	mkdir -p $(OutDir)
	ar rcs $(OutDir)/$(ProjectName).a $(OBJ)

$(OBJDIRS):
	mkdir -p $@ 

.PHONY: clean

clean:
	rm -rf $(IntDir)
