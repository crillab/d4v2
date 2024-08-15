TARGET_EXEC ?= bipe

BUILD_DIR ?= build
SRC_DIRS ?= src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)
INCLUDES := -I../

CPPFLAGS ?= -MMD -MP -O3 -static
LIBS := -L../build/ -lbipe ../3rdParty/glucose-3.0/core/libglucose.a -lboost_program_options 

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS) ../build/libbipe.a
	@echo Linking: "$@ ( $(foreach f,$^,$(subst $(BUILD_DIR)/$(SRC_DIRS),,$f)) )"	
	@$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(LIBS) $(INCLUDES) -static -O3 
	
$(BUILD_DIR)/%.cpp.o: %.cpp
	@echo Compiling: $<
	@$(MKDIR_P) $(dir $@)
	@$(CXX) -fPIC $(CPPFLAGS) $(CFLAGS) -c $< -o $@ $(INCLUDES)

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p
