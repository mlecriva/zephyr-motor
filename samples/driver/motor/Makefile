include Makefile.variable

.PHONY: *

build: ## Build the zephyr firmware [BOARD] [BUILD_DIR] [ZEPHYR_BASE]
	west build -p auto -b $(BOARD) --build-dir=$(BUILD_DIR) --sysbuild

flash:  ## Flash the built firmware [BUILD_DIR] [FLASH_EXTRA_ARGS]
	west flash --build-dir=$(BUILD_DIR)

debug: ## Flash and debug the built firmware [APP] [BUILD_DIR]
	west debug --build-dir=$(BUILD_DIR) --domain .

initlevels: ## Show initialization levels for the built firmware [APP] [BUILD_DIR]
	west build -t initlevels --build-dir=$(BUILD_DIR) --domain .

clean:  ## Remove Zephyr build directory [BUILD_DIR]
	rm -rf $(BUILD_DIR)
