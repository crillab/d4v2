EXEC ?= compiler

BUILD_DIR ?= build
SRC_DIRS ?= src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

CPPFLAGS ?= -MMD -MP

ifeq ($(shell uname),Darwin)
# macOS: Homebrew GNU toolchain.  boost::program_options is linked from our OWN g++
# (libstdc++) build of it -- Homebrew's libboost_program_options.a is clang/libc++
# and ABI-incompatible with g++.  gmp/gmpxx are C-ABI (fine), from Homebrew.
CXX := g++-16
LIBS := ../../build/libd4.a ../../3rdParty/boost_po/libboost_program_options.a -L/opt/homebrew/lib -lz -lgmpxx -lgmp
INCLUDES := -I../../ -I/opt/homebrew/include
else
LIBS := -Wl,-R../../build/ ../../build/libd4.a -lboost_program_options -lz -lgmpxx -lgmp
INCLUDES := -I../../
endif

c: $(BUILD_DIR)/$(EXEC)
p: $(BUILD_DIR)/$(EXEC)_profile
d: $(BUILD_DIR)/$(EXEC)_debug
s: $(BUILD_DIR)/$(EXEC)_static

$(BUILD_DIR)/$(EXEC): CFLAGS += -O3
$(BUILD_DIR)/$(EXEC)_profile: CFLAGS += -O2 -pg
$(BUILD_DIR)/$(EXEC)_debug: CFLAGS += -O2 -g
$(BUILD_DIR)/$(EXEC)_static: CFLAGS += -O3 -DNDEBUG

$(BUILD_DIR)/$(EXEC):	LDFLAGS += 
$(BUILD_DIR)/$(EXEC)_profile:	LDFLAGS += -pg
$(BUILD_DIR)/$(EXEC)_debug:	LDFLAGS += -g
$(BUILD_DIR)/$(EXEC)_static:	LDFLAGS += -static


$(BUILD_DIR)/$(EXEC) $(BUILD_DIR)/$(EXEC)_profile $(BUILD_DIR)/$(EXEC)_debug $(BUILD_DIR)/$(EXEC)_static: $(OBJS) ../../build/libd4.a
	@echo Linking: "$@ ( $(foreach f,$^,$(subst $(BUILD_DIR)/$(SRC_DIRS),,$f)) )"	
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(INCLUDES) $(LIBS)
	
$(BUILD_DIR)/%.cpp.o: %.cpp
	@echo Compiling: $<
	@$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CFLAGS) -c $< -o $@ $(INCLUDES)


.PHONY: clean 

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p
