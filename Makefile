VENV := .venv/bin
PYTHON := $(VENV)/python
PYTEST := $(VENV)/pytest
RUFF := $(VENV)/ruff
BLACK := $(VENV)/black

.PHONY: test lint format check tree env

test:
	$(PYTEST) -q

lint:
	$(RUFF) check src tests scripts

format:
	$(BLACK) src tests scripts

check: lint test

tree:
	tree -L 4 -I ".venv|__pycache__|.git|logs|data"

env:
	$(PYTHON) scripts/check_env.py
