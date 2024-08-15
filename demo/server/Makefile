EXEC ?= server

BUILD_DIR ?= build
SRC_DIRS ?= src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

CPPFLAGS ?= -MMD -MP
LIBS := -Wl,-R../../build/ ../../build/libd4.a -lboost_program_options -lz -lgmpxx -lgmp 
INCLUDES := -I../../

s: $(BUILD_DIR)/$(EXEC)
p: $(BUILD_DIR)/$(EXEC)_profile
d: $(BUILD_DIR)/$(EXEC)_debug
rs: $(BUILD_DIR)/$(EXEC)_static

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
