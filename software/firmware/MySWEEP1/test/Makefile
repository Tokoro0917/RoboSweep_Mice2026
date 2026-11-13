# PC 上でテストする
#   test_motion: ctl_motion.c / ctl_wall.c
#   test_maze:   maze.c
#   test_auto:   app_auto.c（maze.c と ctl_motion.c も本物を使う）
CFLAGS = -std=gnu11 -Wall -Wextra -O0 -g -Istub -I. -I../Core/Inc
MOTION_SRCS = test_motion.c fake_hw.c ../Core/Src/ctl_motion.c ../Core/Src/ctl_wall.c
MAZE_SRCS = test_maze.c sim_maze.c ../Core/Src/maze.c
AUTO_SRCS = test_auto.c sim_maze.c fake_hw.c ../Core/Src/app_auto.c ../Core/Src/maze.c ../Core/Src/ctl_motion.c ../Core/Src/ctl_wall.c

all: test_motion test_maze test_auto
	./test_motion
	./test_maze
	./test_auto

test_motion: $(MOTION_SRCS)
	$(CC) $(CFLAGS) $(MOTION_SRCS) -lm -o $@

test_maze: $(MAZE_SRCS) sim_maze.h ../Core/Inc/maze.h
	$(CC) $(CFLAGS) $(MAZE_SRCS) -o $@

test_auto: $(AUTO_SRCS) ../Core/Inc/maze.h ../Core/Inc/app_auto.h ../Core/Inc/config.h
	$(CC) $(CFLAGS) $(AUTO_SRCS) -lm -o $@

clean:
	rm -f test_motion test_maze test_auto
.PHONY: all clean
