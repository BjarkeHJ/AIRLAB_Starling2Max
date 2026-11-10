PY ?= python3

.DEFAULT_GOAL := help
.PHONY: help generate check test

help:
	@echo "Available commands:"
	@echo "make generate	- Build all files"
	@echo "make check	- Check for stale files"

generate:
	$(PY) -m starling2max.generate

check:
	$(PY) -m starling2max.generate --check

test:
	$(PY) -m pytest -q tests

