report: clean
	latexmk -f report.tex

clean: clean_keep_pdf
	rm -f report.pdf

clean_keep_pdf:
	latexmk -c report.tex
	rm -f report.bbl