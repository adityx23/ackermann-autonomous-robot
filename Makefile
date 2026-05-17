.PHONY: test lint format check tree env

test:
	pytest -q

lint:
	ruff check src tests scripts

format:
	black src tests scripts

check: lint test

tree:
	tree -L 4 -I ".venv|__pycache__|.git|logs|data"

env:
	python scripts/check_env.py
