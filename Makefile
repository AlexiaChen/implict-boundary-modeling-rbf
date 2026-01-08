.PHONY: all build clean rebuild run test-data help

# Default target
all: build

# Configure and build (out-of-source)
build:
	@echo "Configuring CMake..."
	@mkdir -p build
	@cd build && cmake ..
	@echo "Building..."
	@cd build && $(MAKE) -j$$(nproc)
	@echo "Build complete! Binaries are in build/bin/"

# Clean build artifacts
clean:
	@echo "Cleaning build directory..."
	@rm -rf build
	@echo "Clean complete."

# Rebuild from scratch
rebuild: clean build

# Run the main application
run: build
	@echo "Running RBF Implicit Boundary Reconstruction..."
	@./build/bin/rbf_implicit

# Generate test data
test-data: build
	@echo "Generating test data..."
	@cd build/bin && ./generate_test_data
	@echo "Test data generated in build/bin/"

# Show help
help:
	@echo "RBF Implicit Boundary Reconstruction - Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make              - Alias for 'make build'"
	@echo "  make build        - Configure and build the project"
	@echo "  make clean        - Remove build artifacts"
	@echo "  make rebuild      - Clean and rebuild from scratch"
	@echo "  make run          - Build and run the main application"
	@echo "  make test-data    - Build and generate test data"
	@echo "  make help         - Show this help message"
	@echo ""
	@echo "Build output:"
	@echo "  build/bin/rbf_implicit       - Main application"
	@echo "  build/bin/generate_test_data - Test data generator"
	@echo ""
	@echo "Example usage:"
	@echo "  make              # Build the project"
	@echo "  make run          # Build and run"
	@echo "  make test-data    # Generate test data (two_spheres.pcd + .labels)"
