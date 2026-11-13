# PC 上でテストする
#   test_motion: ctl_motion.c / ctl_wall.c
#   test_maze:   maze.c
CFLAGS = -std=gnu11 -Wall -Wextra -O0 -g -Istub -I. -I../Core/Inc
MOTION_SRCS = test_motion.c fake_hw.c ../Core/Src/ctl_motion.c ../Core/Src/ctl_wall.c
MAZE_SRCS = test_maze.c ../Core/Src/maze.c

all: test_motion test_maze
	./test_motion
	./test_maze

test_motion: $(MOTION_SRCS)
	$(CC) $(CFLAGS) $(MOTION_SRCS) -lm -o $@

test_maze: $(MAZE_SRCS) ../Core/Inc/maze.h
	$(CC) $(CFLAGS) $(MAZE_SRCS) -o $@

clean:
	rm -f test_motion test_maze
.PHONY: all clean
