.PHONY: build run clean
build:
	python3 -m py_compile src/main.py
run:
	python3 src/main.py
clean:
	rm -rf src/__pycache__
