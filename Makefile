# 編譯器設定
CC = gcc
CFLAGS = -Wall -O2

# 目標檔名
TARGET = evg

# 所有的來源檔與物件檔
SRCS = ev.c battery.c mechanic.c vehicle.c solar.c throttle.c free.c memalloc.c power.c
OBJS = $(SRCS:.c=.o)

# 編譯規則
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET)
