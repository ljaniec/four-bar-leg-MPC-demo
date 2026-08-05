.PHONY: setup demo figures presentation check clean

setup:
	./scripts/setup.sh

demo:
	./scripts/run_demo.sh

figures:
	./scripts/regenerate_presentation_figures.sh

presentation:
	./scripts/build_presentation.sh

check:
	./scripts/check.sh

clean:
	rm -rf artifacts build dist *.egg-info src/*.egg-info .pytest_cache .ruff_cache
	$(MAKE) -C presentation clean
