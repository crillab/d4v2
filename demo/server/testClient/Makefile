TARGET_EXEC ?= client

BUILD_DIR ?= build
SRC_DIRS ?= src

SRCS := $(shell find $(SRC_DIRS) -name *.c)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

CPPFLAGS ?= -MMD -MP -g

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	@echo Linking: "$@ ( $(foreach f,$^,$(subst $(BUILD_DIR)/$(SRC_DIRS),,$f)) )"	
	@$(CC) $(OBJS) -o $@ $(LDFLAGS)
	
$(BUILD_DIR)/%.c.o: %.c
	@echo Compiling: $<
	@$(MKDIR_P) $(dir $@)
	@$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p
