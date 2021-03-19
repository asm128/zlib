Platform:=Linux.x64
ProjectName:=zlibvc
Configuration:=Release

OPT=3
ifeq ($(DEBUG), 1)
	Configuration=Debug
	OPT=0
endif

OutDir =../$(Platform).$(Configuration)
IntDir =../obj/$(Platform).$(Configuration)/$(ProjectName)

OBJDIRS := $(patsubst %, $(IntDir), $(ProjectName))

CFLAGS=-std=c17 -Wall -O$(OPT) 
CC=gcc

_OBJ = $(patsubst %.c,%.o,$(wildcard *.c)) 
OBJ = $(patsubst %,$(IntDir)/%,$(_OBJ))

$(IntDir)/%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

build: $(OBJDIRS) $(OBJ)
	ar rcs $(OutDir)/$(ProjectName).a $(OBJ)

$(OBJDIRS):
	mkdir -p $@ 

.PHONY: clean

clean:
	rm -rf $(IntDir)
