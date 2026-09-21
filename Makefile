.PHONY: build run clean test
build:
	python3 -m py_compile src/*.py
run:
	python3 src/main.py
test:
	python3 -m unittest discover -s tests -v
clean:
	rm -rf src/__pycache__ tests/__pycache__
