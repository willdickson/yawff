LIBRARY_NAME=yawff
MINOR_NUM=1
RELEASE_NUM=0.1
INSTALL_DIR=/usr/local/lib

SRC=${LIBRARY_NAME}.c check.c util.c test-util.c
OBJ=${SRC:.c=.o} 
TARGET=lib${LIBRARY_NAME}.so.${MINOR_NUM}

CC= $(shell rtai-config --cc)
LXRT_CFLAGS= $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS= $(shell rtai-config --lxrt-ldflags)
TEST_LDFLAGS= -lcunit
INCLUDES= -I 
CFLAGS= -fPIC -c

all: $(TARGET) test devel

$(TARGET): $(OBJ)
	${CC} -shared -W1,-soname,$@ -o $@.$(RELEASE_NUM) $? -lm $(LXRT_LDFLAGS)

$(OBJ): $(SRC)	
	$(CC) $(CFLAGS) $(LXRT_CFLAGS) $?  

test: $(SRC) test-$(LIBRARY_NAME).c
	$(CC) $(LXRT_CFLAGS) $? -o test-$(LIBRARY_NAME) $(TEST_LDFLAGS) -lm $(LXRT_LDFLAGS)

devel: $(SRC) devel-$(LIBRARY_NAME).c
	$(CC) $(LXRT_CFLAGS) $? -o devel-$(LIBRARY_NAME) $(TEST_LDFLAGS) -lm $(LXRT_LDFLAGS)

.PHONY: clean install uninstall

clean: 
	if [ -f $(TARGET).$(RELEASE_NUM) ]; then rm $(TARGET).$(RELEASE_NUM); fi
	if [ -f test-$(LIBRARY_NAME) ]; then rm test-$(LIBRARY_NAME); fi
	if [ -f devel-$(LIBRARY_NAME) ]; then rm devel-$(LIBRARY_NAME); fi
	-rm *.o
	-rm *~

install:
	cp $(TARGET).$(RELEASE_NUM) $(INSTALL_DIR)
	if [ ! -h $(INSTALL_DIR)/$(TARGET) ]; then ln -s ${INSTALL_DIR}/$(TARGET).$(RELEASE_NUM) ${INSTALL_DIR}/$(TARGET); fi
	ldconfig

uninstall:
	rm ${INSTALL_DIR}/$(TARGET)
	rm ${INSTALL_DIR}/$(TARGET).$(RELEASE_NUM)
	ldconfig	