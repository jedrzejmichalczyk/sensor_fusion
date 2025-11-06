# Makefile for LaTeX documentation

DOC = sensor_fusion_documentation
BIB = references

all: $(DOC).pdf

$(DOC).pdf: $(DOC).tex $(BIB).bib
	pdflatex $(DOC).tex
	bibtex $(DOC)
	pdflatex $(DOC).tex
	pdflatex $(DOC).tex

clean:
	rm -f *.aux *.log *.bbl *.blg *.toc *.out *.lot *.lof

cleanall: clean
	rm -f $(DOC).pdf

.PHONY: all clean cleanall
