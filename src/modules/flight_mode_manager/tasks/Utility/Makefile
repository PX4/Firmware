
.PHONY: all tests clean
all: test_velocity_smoothing

test_velocity_smoothing: test_velocity_smoothing.cpp VelocitySmoothing.cpp
	@g++ $^ -std=c++11 -I ../../../ -o $@

tests: test_velocity_smoothing
	@echo "Test velocity smoothing"

clean:
	@rm test_velocity_smoothing
