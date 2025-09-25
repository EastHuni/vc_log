# 컴파일러/옵션
CXX      = g++
CXXFLAGS = -std=gnu++17 -O2 -Wall -Wextra -Wpedantic -MMD -MP

# 타깃/소스
TARGET   = main
SRCS     = main.cpp
OBJS     = $(SRCS:.cpp=.o)
DEPS     = $(OBJS:.o=.d)

# 기본 목표
.PHONY: all clean
all: $(TARGET)

# 링크
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^

# 컴파일 (.cpp -> .o)
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 자동 의존성 포함
-include $(DEPS)

# 청소
clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)
