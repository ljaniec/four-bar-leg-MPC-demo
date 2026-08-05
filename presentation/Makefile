TEX := mpc_dla_robota_czworonoznego.tex
PDF := mpc_dla_robota_czworonoznego.pdf
ENGINE ?= xelatex
PYTHON ?= python

GENERATED_FIGURES := \
	figures/leg_three_motor_roles_annotated.png \
	figures/four_bar_extension_side_view.png \
	figures/mpc_leg_trajectory.png \
	figures/mpc_joint_states.png \
	figures/mpc_safety_margins.png

.PHONY: pdf figures clean

pdf: $(PDF)

figures: $(GENERATED_FIGURES)

$(GENERATED_FIGURES):
	PYTHONPATH=../src $(PYTHON) -m four_bar_leg_mpc \
		--output-dir figures \
		--language pl \
		--no-animation

$(PDF): $(TEX) $(GENERATED_FIGURES)
	$(ENGINE) -interaction=nonstopmode -halt-on-error $(TEX)
	$(ENGINE) -interaction=nonstopmode -halt-on-error $(TEX)

clean:
	rm -f *.aux *.log *.nav *.out *.snm *.toc *.vrb *.fls *.fdb_latexmk *.synctex.gz

clean-generated: clean
	rm -f figures/*.png $(PDF)
